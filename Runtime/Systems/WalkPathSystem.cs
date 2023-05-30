using Unity.Burst;
using Unity.Burst.Intrinsics;
using Unity.Collections;
using Unity.Entities;
using Unity.Mathematics;
using Unity.Physics.Authoring;
using Unity.Transforms;

namespace NZCore.Pathfinding
{
    public struct PathFinderControlled : IComponentData, IEnableableComponent { }
    
    [BurstCompile]
    [UpdateInGroup(typeof(Stage1SystemGroup))]
    [UpdateAfter(typeof(PathfinderSystem))]
    public partial struct WalkPathSystem : ISystem
    {
        private EntityQuery query;

        [BurstCompile]
        public void OnCreate(ref SystemState state)
        {
            query = new EntityQueryBuilder(Allocator.Temp)
                .WithAll<FindPath, PathBuffer>()
                .Build(ref state);
        }
        
        public void OnDestroy(ref SystemState state)
        {
        }

        [BurstCompile]
        public void OnUpdate(ref SystemState state)
        {
            var deltaTime = SystemAPI.Time.DeltaTime;
            var destroyCommandBuffer = SystemAPI.GetSingleton<DestroyEntityCommandBufferSystem.Singleton>().CreateCommandBuffer(state.WorldUnmanaged);
            var pathResults = SystemAPI.GetSingleton<PathResultsSingleton>();

            var processHandle = new ProcessPathResults()
            {
                FindPath_ReadHandle = SystemAPI.GetComponentTypeHandle<FindPath>(true),
                PathBuffer_WriteHandle = SystemAPI.GetBufferTypeHandle<PathBuffer>(false),
                PathResults = pathResults.Results,

            }.ScheduleParallel(query, state.Dependency);

            state.Dependency = new WalkPathJob
            {
                commandBuffer = destroyCommandBuffer.AsParallelWriter(),
                deltaTime =  deltaTime,
                Entities_ReadHandle = SystemAPI.GetEntityTypeHandle(),
                FindPath_WriteHandle = SystemAPI.GetComponentTypeHandle<FindPath>(false),
                PathBuffer_ReadHandle = SystemAPI.GetBufferTypeHandle<PathBuffer>(true),
                //PathFinderControlled_WriteHandle = SystemAPI.GetComponentTypeHandle<PathFinderControlled>(false),
                PathFinderControlled_WriteLookup = SystemAPI.GetComponentLookup<PathFinderControlled>(false),

                LocalTransform_WriteLookup = SystemAPI.GetComponentLookup<LocalTransform>(false),
                Velocity_WriteLookup = SystemAPI.GetComponentLookup<Velocity>(false)
            }.ScheduleParallel(query, processHandle);
        }
        
        [BurstCompile]
        public unsafe struct ProcessPathResults : IJobChunk
        {
            [ReadOnly] public ComponentTypeHandle<FindPath> FindPath_ReadHandle;
            
            [NativeDisableParallelForRestriction]
            public BufferTypeHandle<PathBuffer> PathBuffer_WriteHandle;
            
            [ReadOnly] public NativeParallelHashMap<int, PathQueryResult> PathResults;

            public void Execute(in ArchetypeChunk chunk, int unfilteredChunkIndex, bool useEnabledMask, in v128 chunkEnabledMask)
            {
                var findPaths = chunk.GetComponentDataPtrRO(ref FindPath_ReadHandle);
                var pathBufferAccessor = chunk.GetBufferAccessor(ref PathBuffer_WriteHandle);
                
                for (int i =0; i < chunk.Count;i++)
                {
                    ref var findPath = ref findPaths[i]; 
                    
                    if (findPath.pathId == 0 || !PathResults.ContainsKey(findPath.pathId))
                        continue;

                    var result = PathResults[findPath.pathId];
                    findPath.PathStatus = PathStatus.Success;
                    
                    //Debug.Log($"Found path result writing {result.PathLength}");
                    
                    var buffer = pathBufferAccessor[i];
                    buffer.Clear();

                    for (int ii = 1; ii < result.PathLength; ii++)
                    {
                        buffer.Add(new PathBuffer()
                        {
                            position = result.Path[ii].position
                        });
                        
                        PhysicsDebugDisplaySystem.Line(result.Path[ii - 1].position, result.Path[ii].position, Unity.DebugDisplay.ColorIndex.Green);
                    }
                }
            }
        }

        [BurstCompile]
        public unsafe struct WalkPathJob : IJobChunk
        {
            [ReadOnly] public EntityTypeHandle Entities_ReadHandle;
            [ReadOnly] public BufferTypeHandle<PathBuffer> PathBuffer_ReadHandle;
            public ComponentTypeHandle<FindPath> FindPath_WriteHandle;
            //public ComponentTypeHandle<PathFinderControlled> PathFinderControlled_WriteHandle;

            [NativeDisableParallelForRestriction] public UnsafeComponentLookup<PathFinderControlled> PathFinderControlled_WriteLookup;
            [NativeDisableParallelForRestriction] public UnsafeComponentLookup<LocalTransform> LocalTransform_WriteLookup;
            [NativeDisableParallelForRestriction] public UnsafeComponentLookup<Velocity> Velocity_WriteLookup;

            public EntityCommandBuffer.ParallelWriter commandBuffer;
            public float deltaTime;

            public void Execute(in ArchetypeChunk chunk, int unfilteredChunkIndex, bool useEnabledMask, in v128 chunkEnabledMask)
            {
                var entities = chunk.GetNativeArray(Entities_ReadHandle);
                var pathBuffers = chunk.GetBufferAccessor(ref PathBuffer_ReadHandle);
                var findPaths = chunk.GetComponentDataPtrRW(ref FindPath_WriteHandle);

                for (int i = 0; i < chunk.Count; i++)
                {
                    ref var findPath = ref findPaths[i];
                    if (findPath.PathStatus != PathStatus.Success)
                        continue;
                    
                    var source = findPath.source;
                    
                    if (!LocalTransform_WriteLookup.HasComponent(source))
                    {
                        // entity was destroyed
                        //Debug.Log($"entity was destroyed");
                        commandBuffer.DestroyEntity(unfilteredChunkIndex, entities[i]);
                        continue;
                    }
                    
                    var pathBuffer = pathBuffers[i];
                    ref var velocity = ref Velocity_WriteLookup.GetRef(source, true);

                    if (findPath.pathWalkerIndex >= pathBuffer.Length)
                    {
                        // reached destination
                        //Debug.Log($"reached destination targetPos: {findPath.targetPosition}");
                        commandBuffer.DestroyEntity(unfilteredChunkIndex, entities[i]);
                        
                        PathFinderControlled_WriteLookup.SetComponentEnabled(source, false);
                        velocity = default;
                        //Debug.Log("Lost control is false");
                        continue;
                    }

                    ref var sourceTransform = ref LocalTransform_WriteLookup.GetRef(source, true);
                    var direction = pathBuffer[findPath.pathWalkerIndex].position - sourceTransform.Position;
                    var lengthToWaypoint = math.lengthsq(direction);
                    var minimumMoveStep = findPath.speed * deltaTime;
                    
                    if (lengthToWaypoint < math.pow(minimumMoveStep, 2))
                    {
                        findPath.pathWalkerIndex++;
                        
                        if (findPath.pathWalkerIndex < pathBuffer.Length)
                            direction = pathBuffer[findPath.pathWalkerIndex].position - sourceTransform.Position;
                    }
                    
                    var normalizedDir = math.normalizesafe(direction);
                    //Debug.Log($"Direction {direction} speed: {findPath.speed}");

                    velocity.Value = normalizedDir * findPath.speed;
                    sourceTransform.Position += normalizedDir * minimumMoveStep;
                    sourceTransform.Rotation = quaternion.LookRotation(normalizedDir, new float3(0, 1, 0));

                    //Debug.Log($"Walkpath moving by {(normalizedDir * maximumMoveLength)}");
                    PathFinderControlled_WriteLookup.SetComponentEnabled(source, true);
                }
            }
        }
    }
}
