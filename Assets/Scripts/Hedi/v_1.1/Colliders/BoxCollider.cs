using UnityEngine;
using CustomPhysics.v_1_1.Objects;
using CustomPhysics.v_1_1.Physics;

namespace CustomPhysics.v_1_1.Colliders
{
    public class BoxCollider : CustomCollider
    {
        [SerializeField] public Vector3 size = Vector3.one; // Changé à public
        
        public Vector3 Size => Vector3.Scale(size, transform.lossyScale);
        
        public override void CalculateBounds()
        {
            Bounds = new CustomPhysics.v_1_1.Geometry.Bounds(transform.position + centerOffset, Size);
        }
        
        public override bool CheckCollision(CustomCollider other, out CollisionManifold manifold)
        {
            manifold = new CollisionManifold();
            return false;
        }
        
        public override void DrawGizmos()
        {
            Gizmos.color = isTrigger ? Color.yellow : Color.green;
            Gizmos.DrawWireCube(Center, Size);
        }
        
        void OnValidate()
        {
            size.x = Mathf.Max(size.x, 0.001f);
            size.y = Mathf.Max(size.y, 0.001f);
            size.z = Mathf.Max(size.z, 0.001f);
            type = ColliderType.Box;
        }
    }
}