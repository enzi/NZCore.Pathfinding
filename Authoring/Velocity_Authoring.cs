using Unity.Entities;
using Unity.Mathematics;
using UnityEngine;

namespace NZCore.Pathfinding.Authoring
{
    public class Velocity_Authoring : MonoBehaviour
    {
        public float3 velocity;

        public class Velocity_Authoring_Baker : Baker<Velocity_Authoring>
        {
            public override void Bake(Velocity_Authoring authoring)
            { 
                var entity = GetEntity(TransformUsageFlags.Dynamic);
                
                AddComponent(entity, new Velocity()
                {
                    Value = authoring.velocity
                });
            }
        }
    }
}