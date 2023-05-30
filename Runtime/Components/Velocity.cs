using Unity.Entities;
using Unity.Mathematics;

namespace NZCore.Pathfinding
{
    public struct Velocity : IComponentData
    {
        public float3 Value;
    }
}