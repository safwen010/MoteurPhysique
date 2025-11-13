using UnityEngine;
using CustomPhysics.v_1_1.Objects;
using CustomPhysics.v_1_1.Physics;

namespace CustomPhysics.v_1_1.Colliders
{
    public class SphereCollider : CustomCollider
    {
        [SerializeField] public float radius = 1.0f; // Changé à public
        
        public float Radius => radius * Mathf.Max(transform.lossyScale.x, transform.lossyScale.y, transform.lossyScale.z);
        
        public override void CalculateBounds()
        {
            float worldRadius = Radius;
            Bounds = new CustomPhysics.v_1_1.Geometry.Bounds(transform.position + centerOffset, Vector3.one * worldRadius * 2f);
        }
        
        public override bool CheckCollision(CustomCollider other, out CollisionManifold manifold)
        {
            manifold = new CollisionManifold();
            return false;
        }
        
        public override void DrawGizmos()
        {
            Gizmos.color = isTrigger ? Color.yellow : Color.green;
            Gizmos.DrawWireSphere(Center, Radius);
        }
        
        void OnValidate()
        {
            radius = Mathf.Max(radius, 0.001f);
            type = ColliderType.Sphere;
        }
    }
}