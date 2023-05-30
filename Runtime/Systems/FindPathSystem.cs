using System.Resources;
using Unity.Burst;
using Unity.Collections;
using Unity.Entities;
using Unity.Burst.Intrinsics;
using Unity.Entities.Content;
using Unity.Mathematics;
using Unity.Transforms;
using UnityEngine;
using UnityEngine.Experimental.AI;

namespace NZCore.Pathfinding
{
    public struct FindPath : IComponentData
    {
        public Entity source;
        public Entity target;
        
        public float3 targetPosition;
        
        public float speed;
        public float requiredMinDistanceSq;

        public int pathId;
        public PathStatus PathStatus;
        public int pathWalkerIndex;
    }

    [InternalBufferCapacity(0)]
    public struct PathBuffer : IBufferElementData
    {
        public float3 position;
    }

    [CreateAfter(typeof(PathfinderSystem))]
    [UpdateInGroup(typeof(Stage1SystemGroup))]
    public unsafe partial struct FindPathSystem : ISystem
    {
        private EntityQuery query;
        
        private NavMeshWorld _navMeshWorld;
        private NavMeshQuery _navMeshQuery;
        
        NativeArray<float> m_Costs;

        [BurstCompile]
        public void OnCreate(ref SystemState state)
        {
            query = new EntityQueryBuilder(Allocator.Temp)
                .WithAll<FindPath, PathBuffer>()
                .Build(ref state);
            
            _navMeshWorld = NavMeshWorld.GetDefaultWorld();
            _navMeshQuery = new NavMeshQuery(_navMeshWorld, Allocator.Persistent, 0);
            
            m_Costs = new NativeArray<float>(32, Allocator.Persistent);
            for (var i = 0; i < m_Costs.Length; ++i)
                m_Costs[i] = 1.0f;

            SystemAPI.GetSingletonRW<PathRequestsSingleton>();
        }

        [BurstCompile]
        public void OnDestroy(ref SystemState state)
        {
            _navMeshQuery.Dispose();
            m_Costs.Dispose();
        }

        [BurstCompile]
        public void OnUpdate(ref SystemState state)
        {
            var commandBuffer = SystemAPI.GetSingleton<DestroyEntityCommandBufferSystem.Singleton>().CreateCommandBuffer(state.WorldUnmanaged);
            var pathRequests = SystemAPI.GetSingleton<PathRequestsSingleton>();

            pathRequests.Requests.Update();

            state.Dependency = new FindPathJob()
            {
                CommandBuffer = commandBuffer.AsParallelWriter(),
                Entities_ReadHandle = SystemAPI.GetEntityTypeHandle(),
                PathRequests = pathRequests.Requests.AsParallelWriter(),
                
                LocalTransform_Lookup = SystemAPI.GetComponentLookup<LocalTransform>(true),
                NavMeshQuery = _navMeshQuery,
                FindPath_WriteHandle = SystemAPI.GetComponentTypeHandle<FindPath>(false)
            }.ScheduleParallel(query, state.Dependency);
        }
        
        [BurstCompile]
        private struct FindPathJob : IJobChunk
        {
            [ReadOnly] public NavMeshQuery NavMeshQuery;
            public NativeWorkQueue<PathQuery>.ParallelWriter PathRequests;
            public EntityCommandBuffer.ParallelWriter CommandBuffer;

            [ReadOnly] public EntityTypeHandle Entities_ReadHandle;
            public ComponentTypeHandle<FindPath> FindPath_WriteHandle;

            [ReadOnly] public ComponentLookup<LocalTransform> LocalTransform_Lookup;

            public void Execute(in ArchetypeChunk chunk, int unfilteredChunkIndex, bool useEnabledMask, in v128 chunkEnabledMask)
            {
                var entities = chunk.GetEntityDataPtrRO(Entities_ReadHandle);
                var findPaths = chunk.GetComponentDataPtrRW(ref FindPath_WriteHandle);
                
                for (int i =0; i < chunk.Count;i++)
                {
                    ref var findPath = ref findPaths[i];

                    if (!LocalTransform_Lookup.TryGetComponent(findPath.source, out var sourceTransform))
                    {
                        //Debug.Log("source dead -> destroy findPath entity");
                        var entity = entities[i];
                        CommandBuffer.DestroyEntity(unfilteredChunkIndex, entity);
                        continue;
                    }

                    float3 sourcePos = sourceTransform.Position;
                    float3 targetPos = findPath.target == Entity.Null ? findPath.targetPosition : LocalTransform_Lookup[findPath.target].Position; 

                    float length = math.distancesq(sourcePos, targetPos);
                    if (length <= findPath.requiredMinDistanceSq)
                    {
                        // don't destroy, WalkPathSystem needs to give control back
                        continue;
                    }

                    int pathId = PathRequests.TryAdd(out PathQuery* pathRequest);

                    if (pathId == 0)
                    {
                        //Debug.LogError("Too much requests");
                        continue;
                    }

                    //Debug.Log($"Firing off request with id {pathId} from {sourcePos} to {targetPos}");
                    findPath.pathId = pathId;
                    findPath.PathStatus = NZCore.Pathfinding.PathStatus.InProgress;
                    findPath.pathWalkerIndex = 0;
                    
                    pathRequest->PathRequestId = pathId;
                    pathRequest->From = NavMeshQuery.MapLocation(sourcePos, new float3(10, 10, 10), 0, 1 << 0);
                    pathRequest->To = NavMeshQuery.MapLocation(targetPos, new float3(10, 10, 10), 0, 1 << 0);
                    pathRequest->AreaMask = 1 << 0;
                    pathRequest->Status = default; // todo, actual danger, not setting this screws up every request afterwards
                }
            }
        }
    }

}
