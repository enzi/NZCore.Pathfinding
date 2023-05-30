using System;
using Unity.Burst;
using Unity.Collections;
using Unity.Collections.LowLevel.Unsafe;
using Unity.Entities;
using Unity.Jobs;
using Unity.Mathematics;
using UnityEngine.Experimental.AI;

namespace NZCore.Pathfinding
{
    public enum PathStatus
    {
        None,
        InProgress,
        Success,
        Failed
    }

    public unsafe struct PathQueryResult
    {
        public NavMeshLocation* Path;
        public int PathLength;
        //public PathStatus Status;
    }

    public unsafe struct PathQueryPtr
    {
        public PathQuery* PathQuery;
    }

    public unsafe struct PathQuery
    {
        public int PathRequestId;
        
        public NavMeshLocation From;
        public NavMeshLocation To;
        
        //public int id;
        //public int key;
        public int AreaMask;

        // result data
        public NavMeshLocation* Path;
        public int PathLength;
        public PathQueryStatus Status;

    }

    public struct PathRequestsSingleton : IComponentData
    {
        public NativeWorkQueue<PathQuery> Requests;
    }
    
    public struct PathResultsSingleton : IComponentData
    {
        public NativeParallelHashMap<int, PathQueryResult> Results;
    }

    [UpdateInGroup(typeof(Stage1SystemGroup))]
    [UpdateAfter(typeof(FindPathSystem))]
    public partial struct PathfinderSystem : ISystem
    {
        private const int MAX_WORKERS = 100;
        private const int QUERY_MAX_COUNT = 100;
        private const int PATH_MAX_SIZE = 512;
        private const int MAX_PATH_QUEUE_NODES = 4096;
        
        private NavMeshWorld _navMeshWorld;
        private NativeArray<NavMeshQuery> _queries;

        private NativeWorkQueue<PathQuery> _pathRequests; // only for alloc/dispose
        private NativeArray<PathQueryPtr> _inProgress;

        private NativeParallelHashMap<int, PathQueryResult> _pathResults; // only for alloc/dispose


        [BurstCompile]
        public unsafe void OnCreate(ref SystemState state)
        {
            _navMeshWorld = NavMeshWorld.GetDefaultWorld();

            _pathRequests = new NativeWorkQueue<PathQuery>(QUERY_MAX_COUNT, Allocator.Persistent);
            _inProgress = new NativeArray<PathQueryPtr>(QUERY_MAX_COUNT, Allocator.Persistent);
            
            _pathResults = new NativeParallelHashMap<int, PathQueryResult>(QUERY_MAX_COUNT, Allocator.Persistent);

            for (int i = 0; i < QUERY_MAX_COUNT; i++)
            {
                ref var entry = ref _pathRequests.ElementAt(i);
                entry.Path = (NavMeshLocation*) UnsafeUtility.Malloc(UnsafeUtility.SizeOf<NavMeshLocation>() * PATH_MAX_SIZE, UnsafeUtility.AlignOf<NavMeshLocation>(), Allocator.Persistent);
                entry.Status = 0;

                _inProgress[i] = new PathQueryPtr()
                {
                    PathQuery = null
                };
            }

            state.EntityManager.AddComponentData(state.SystemHandle, new PathRequestsSingleton()
            {
                Requests = _pathRequests
            });
            state.EntityManager.AddComponentData(state.SystemHandle, new PathResultsSingleton()
            {
                Results = _pathResults
            });

            _queries = new NativeArray<NavMeshQuery>(MAX_WORKERS, Allocator.Persistent);

            for (int i = 0; i < MAX_WORKERS; i++)
            {
                _queries[i] = new NavMeshQuery(_navMeshWorld, Allocator.Persistent, MAX_PATH_QUEUE_NODES);
            }

            SystemAPI.GetSingletonRW<PathResultsSingleton>();
        }
        
        [BurstCompile]
        public unsafe void OnDestroy(ref SystemState state)
        {
            for (int i = 0; i < QUERY_MAX_COUNT; i++)
            {
                UnsafeUtility.Free(_pathRequests.ElementAt(i).Path, Allocator.Persistent);
            }
            
            _pathRequests.Dispose();

            //results.Dispose();

            for (int i = 0; i < MAX_WORKERS; i++)
            {
                _queries[i].Dispose();
            }

            _queries.Dispose();
            
            _pathResults.Dispose();
            _inProgress.Dispose();
            
        }

        [BurstCompile]
        public void OnUpdate(ref SystemState state)
        {
            var requests = SystemAPI.GetSingleton<PathRequestsSingleton>();
            var results = SystemAPI.GetSingleton<PathResultsSingleton>();
           
            var clearHandle = new ClearResultsHashMap()
            {
                Results = results.Results
            }.Schedule(state.Dependency);
            
            state.Dependency = new ProcessQueues()
            {
                MaxIters = 4096,
                NavMeshQueries = _queries,
                PathRequests = requests.Requests.AsParallelReader(),
                InProgress = _inProgress,
                Results = results.Results.AsParallelWriter()
            //}.ScheduleParallel(MAX_WORKERS, 1, state.Dependency);
            }.Schedule(MAX_WORKERS, clearHandle);
        }

        [BurstCompile]
        private struct ClearResultsHashMap : IJob
        {
            public NativeParallelHashMap<int, PathQueryResult> Results;
            
            public void Execute()
            {
                Results.Clear();
            }
        }
        
        [BurstCompile]
        private unsafe struct ProcessQueues : IJobFor
        {
            public int MaxIters;
            
            [ReadOnly]
            [NativeDisableContainerSafetyRestriction]
            public NativeArray<NavMeshQuery> NavMeshQueries;

            public NativeWorkQueue<PathQuery>.ParallelReader PathRequests;
            public NativeArray<PathQueryPtr> InProgress;
            
            public NativeParallelHashMap<int, PathQueryResult>.ParallelWriter Results;

            public void Execute(int index)
            {
                int iterCount = MaxIters;
                var navMeshQuery = NavMeshQueries[index];

                if (InProgress[index].PathQuery != null)
                {
                    //ref var previousQuery = ref UnsafeUtility.AsRef<PathQuery>(InProgress[index].PathQuery);
                    ref var previousQuery = ref *InProgress[index].PathQuery;
                    
                    if (!ProcessElement(ref previousQuery, ref navMeshQuery, ref iterCount))
                    {
                        // Didn't finish processing the work
                        return;
                    }
                }

                //ref var previousQuery = ref InProgress.ElementAt(index);
                
                if (PathRequests.TryGetNext(out PathQuery* pathQuery))
                {
                    if (!ProcessElement(ref *pathQuery, ref navMeshQuery, ref iterCount))
                    {
                        // Didn't finish processing the work, store it for later
                        InProgress[pathQuery->PathRequestId] = new PathQueryPtr()
                        {
                            PathQuery = pathQuery
                        };
                        return;
                    }
                }

                var ptr = (PathQueryPtr*) InProgress.GetUnsafePtr();
                ref var element = ref UnsafeUtility.AsRef<PathQueryPtr>(ptr + index);
                element.PathQuery = null;
            }
            
            // True if we should continue processing
            private bool ProcessElement(ref PathQuery query, ref NavMeshQuery navMeshQuery, ref int iterCount)
            {
                if (iterCount <= 0)
                {
                    return false;
                }

                var result = Process(ref query, ref navMeshQuery, ref iterCount);
                if (!result)
                {
                    return false;
                }

                // Write our result
                //Debug.Log($"Write path results from id {query.PathRequestId} length {query.PathLength}");
                Results.TryAdd(query.PathRequestId, new PathQueryResult
                {
                    //Status = *query.Status,
                    Path = query.Path,
                    PathLength = query.PathLength,
                });
                //Check.Assume(added);
                return true;
            }
            
            private bool Process(ref PathQuery query, ref NavMeshQuery navMeshQuery, ref int iterCount)
            {
                // Handle query start.
                if (query.Status == 0)
                {
                    query.Status = navMeshQuery.BeginFindPath(query.From, query.To, query.AreaMask, default);
                }

                // Handle query in progress.
                if (query.Status == PathQueryStatus.InProgress)
                {
                    query.Status = navMeshQuery.UpdateFindPath(iterCount, out var iterationsPerformed);
                    iterCount -= iterationsPerformed;
                }

                // Check query complete
                if (query.Status == PathQueryStatus.Success)
                {
                    var unusedResult = navMeshQuery.EndFindPath(out int pathLength);

                    var polygons = new NativeArray<PolygonId>(pathLength, Allocator.Temp);
                    navMeshQuery.GetPathResult(polygons);
                    
                    var straightPathFlags = new NativeArray<StraightPathFlags>(PATH_MAX_SIZE, Allocator.Temp);
                    var vertexSide = new NativeArray<float>(PATH_MAX_SIZE, Allocator.Temp);
                    
                    var cornerCount = 0;

                    query.Status = FindStraightPath(ref navMeshQuery,
                        query.From.position,
                        query.To.position,
                        pathLength,
                        polygons,
                        query.Path,
                        ref straightPathFlags,
                        ref vertexSide,
                        ref cornerCount
                    );

                    query.PathLength = cornerCount;

                    return true;
                }

                // We've still finished, we just failed in our query
                if (query.Status == PathQueryStatus.Failure)
                {
                    //Debug.Log($"PathQueryStatus.Failure id {query.PathRequestId}");
                    return true;
                }

                return false;
            }

            private PathQueryStatus FindStraightPath(
                ref NavMeshQuery query,
                float3 startPos,
                float3 endPos, 
                int pathSize, 
                NativeArray<PolygonId> polygons,
                NavMeshLocation* straightPath, 
                ref NativeArray<StraightPathFlags> straightPathFlags,
                ref NativeArray<float> vertexSide,
                ref int straightPathCount)
            {
                if (!query.IsValid(polygons[0]))
                {
                    //Debug.Log("Query Failure 1");
                    straightPath[0] = new NavMeshLocation (); // empty terminator
                    return PathQueryStatus.Failure; // | PathQueryStatus.InvalidParam;
                }

                straightPath[0] = query.CreateLocation (startPos, polygons[0]);

                straightPathFlags[0] = StraightPathFlags.Start;

                var apexIndex = 0;
                var n = 1;

                if (pathSize > 1)
                {
                    var startPolyWorldToLocal = query.PolygonWorldToLocalMatrix(polygons[0]);

                    float3 apex = startPolyWorldToLocal.MultiplyPoint(startPos);
                    var left = new float3(0, 0, 0); // Vector3.zero accesses a static readonly which does not work in burst yet
                    var right = new float3(0, 0, 0);
                    var leftIndex = -1;
                    var rightIndex = -1;

                    for (var i = 1; i <= pathSize; ++i)
                    {
                        var polyWorldToLocal = query.PolygonWorldToLocalMatrix(polygons[apexIndex]);

                        float3 vl, vr;
                        if (i == pathSize)
                        {
                            vl = vr = polyWorldToLocal.MultiplyPoint(endPos);
                           
                        }
                        else
                        {
                            var success = query.GetPortalPoints(polygons[i - 1], polygons[i], out var vecvl, out var vecvr);
                            if (!success)
                            {
                                //Debug.Log("Query Failure 2");
                                return PathQueryStatus.Failure; // | PathQueryStatus.InvalidParam;
                            }

                            vl = polyWorldToLocal.MultiplyPoint(vecvl);
                            vr = polyWorldToLocal.MultiplyPoint(vecvr);
                        }
                        
                        vl -= apex;
                        vr -= apex;

                        // Ensure left/right ordering
                        if (Perp2D(vl, vr) < 0)
                            Swap(ref vl, ref vr);

                        // Terminate funnel by turning
                        if (Perp2D(left, vr) < 0)
                        {
                            var polyLocalToWorld = query.PolygonLocalToWorldMatrix(polygons[apexIndex]);
                            var termPos = polyLocalToWorld.MultiplyPoint(apex + left);

                            n = RetracePortals(ref query, apexIndex, leftIndex, n, termPos, polygons, straightPath, ref straightPathFlags);
                            if (vertexSide.Length > 0)
                            {
                                vertexSide[n - 1] = -1;
                            }

                            //Debug.Log("LEFT");

                            if (n == PATH_MAX_SIZE)
                            {
                                straightPathCount = n;
                                return PathQueryStatus.Success; // | PathQueryStatus.BufferTooSmall;
                            }

                            apex = polyWorldToLocal.MultiplyPoint(termPos);
                            left = float3.zero;
                            right = float3.zero;
                            i = apexIndex = leftIndex;
                            continue;
                        }

                        if (Perp2D(right, vl) > 0)
                        {
                            var polyLocalToWorld = query.PolygonLocalToWorldMatrix(polygons[apexIndex]);
                            var termPos = polyLocalToWorld.MultiplyPoint(apex + right);

                            n = RetracePortals(ref query, apexIndex, rightIndex, n, termPos, polygons, straightPath, ref straightPathFlags);
                            if (vertexSide.Length > 0)
                            {
                                vertexSide[n - 1] = 1;
                            }

                            //Debug.Log("RIGHT");

                            if (n == PATH_MAX_SIZE)
                            {
                                straightPathCount = n;
                                return PathQueryStatus.Success; // | PathQueryStatus.BufferTooSmall;
                            }

                            apex = polyWorldToLocal.MultiplyPoint(termPos);
                            left = float3.zero;
                            right = float3.zero;
                            i = apexIndex = rightIndex;
                            continue;
                        }

                        // Narrow funnel
                        if (Perp2D(left, vl) >= 0)
                        {
                            left = vl;
                            leftIndex = i;
                        }

                        if (Perp2D(right, vr) <= 0)
                        {
                            right = vr;
                            rightIndex = i;
                        }
                    }
                }

                // Remove the the next to last if duplicate point - e.g. start and end positions are the same
                // (in which case we have get a single point)
                if (n > 0 && math.all(((float3)straightPath[n - 1].position == endPos)))
                    n--;

                n = RetracePortals(ref query, apexIndex, pathSize - 1, n, endPos, polygons, straightPath, ref straightPathFlags);
                if (vertexSide.Length > 0)
                {
                    vertexSide[n - 1] = 0;
                }

                if (n == PATH_MAX_SIZE)
                {
                    straightPathCount = n;
                    return PathQueryStatus.Success; // | PathQueryStatus.BufferTooSmall;
                }

                // Fix flag for final path point
                straightPathFlags[n - 1] = StraightPathFlags.End;

                straightPathCount = n;
                return PathQueryStatus.Success;
            }
            
            private int RetracePortals(
                ref NavMeshQuery query,
                int startIndex,
                int endIndex,
                int n, 
                float3 termPos, 
                NativeArray<PolygonId> path, 
                NavMeshLocation* straightPath,
                ref NativeArray<StraightPathFlags> straightPathFlags)
            {
                for (var k = startIndex; k < endIndex - 1; ++k)
                {
                    var type1 = query.GetPolygonType(path[k]);
                    var type2 = query.GetPolygonType(path[k + 1]);
                
                    if (type1 != type2)
                    {
                        float3 cpa1, cpa2;
                        var status = query.GetPortalPoints(path[k], path[k + 1], out var vecl, out var vecr);
                   
                        SegmentSegmentCPA(out cpa1, out cpa2, vecl, vecr, straightPath[n - 1].position, termPos);
                        straightPath[n] = query.CreateLocation(cpa1, path[k + 1]);

                        straightPathFlags[n] = (type2 == NavMeshPolyTypes.OffMeshConnection) ? StraightPathFlags.OffMeshConnection : 0;
                        if (++n == PATH_MAX_SIZE)
                        {
                            return PATH_MAX_SIZE;
                        }
                    }
                }

                straightPath[n] = query.CreateLocation(termPos, path[endIndex]);
                straightPathFlags[n] = query.GetPolygonType(path[endIndex]) == NavMeshPolyTypes.OffMeshConnection ? StraightPathFlags.OffMeshConnection : 0;
                return ++n;
            }
            
            private float Perp2D(float3 u, float3 v)
            {
                return u.z * v.x - u.x * v.z;
            }
        
            private void Swap (ref float3 a, ref float3 b)
            {
                (a, b) = (b, a);
            }
        
            private bool SegmentSegmentCPA (out float3 c0, out float3 c1, float3 p0, float3 p1, float3 q0, float3 q1)
            {
                var u = p1 - p0;
                var v = q1 - q0;
                var w0 = p0 - q0;

                float a = math.dot (u, u);
                float b = math.dot (u, v);
                float c = math.dot (v, v);
                float d = math.dot (u, w0);
                float e = math.dot (v, w0);

                float den = (a * c - b * b);
                float sc, tc;

                if (den == 0)
                {
                    sc = 0;
                    tc = d / b;

                    // todo: handle b = 0 (=> a and/or c is 0)
                }
                else
                {
                    sc = (b * e - c * d) / (a * c - b * b);
                    tc = (a * e - b * d) / (a * c - b * b);
                }

                c0 = math.lerp (p0, p1, sc);
                c1 = math.lerp (q0, q1, tc);

                return den != 0;
            }
        }
        
        [Flags]
        public enum StraightPathFlags
        {
            Start = 0x01, // The vertex is the start position.
            End = 0x02, // The vertex is the end position.
            OffMeshConnection = 0x04 // The vertex is start of an off-mesh link.
        }
    }
}