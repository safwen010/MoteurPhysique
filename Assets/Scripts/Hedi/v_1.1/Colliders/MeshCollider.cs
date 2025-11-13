using UnityEngine;
using CustomPhysics.v_1_1.Objects;
using CustomPhysics.v_1_1.Physics;

namespace CustomPhysics.v_1_1.Colliders
{
    public class MeshCollider : CustomCollider
    {
        [SerializeField] private UnityEngine.Mesh customMesh;
        [SerializeField] private bool convex = true;
        
        public override void CalculateBounds()
        {
            if (customMesh != null)
            {
                Bounds = new CustomPhysics.v_1_1.Geometry.Bounds(transform.position + centerOffset, customMesh.bounds.size);
            }
            else
            {
                Bounds = new CustomPhysics.v_1_1.Geometry.Bounds(transform.position + centerOffset, Vector3.one);
            }
        }
        
        public override bool CheckCollision(CustomCollider other, out CollisionManifold manifold)
        {
            manifold = new CollisionManifold();
            // Implémentation complexe - GJK/EPA algorithm
            return false;
        }
        
        public override void DrawGizmos()
        {
            if (customMesh != null)
            {
                Gizmos.color = isTrigger ? Color.yellow : Color.green;
                Gizmos.DrawWireMesh(customMesh, Center, transform.rotation, transform.lossyScale);
            }
        }
    }
}