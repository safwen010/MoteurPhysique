using UnityEngine;
using CustomPhysics.v_1_1.Objects;
using CustomPhysics.v_1_1.Colliders;

namespace CustomPhysics.v_1_1.Physics
{
    public static class CollisionDetection
    {
        public static bool CheckCollision(CustomPhysics.v_1_1.Objects.CustomCollider a, CustomPhysics.v_1_1.Objects.CustomCollider b, out CollisionManifold manifold)
        {
            manifold = new CollisionManifold();
            
            // Vérification rapide des bounds
            if (!a.Bounds.Intersects(b.Bounds))
                return false;
            
            // Détection spécifique selon les types
            if (a is CustomPhysics.v_1_1.Colliders.SphereCollider && b is CustomPhysics.v_1_1.Colliders.SphereCollider)
            {
                return SphereVsSphere((CustomPhysics.v_1_1.Colliders.SphereCollider)a, (CustomPhysics.v_1_1.Colliders.SphereCollider)b, out manifold);
            }
            else if (a is CustomPhysics.v_1_1.Colliders.BoxCollider && b is CustomPhysics.v_1_1.Colliders.BoxCollider)
            {
                return BoxVsBox((CustomPhysics.v_1_1.Colliders.BoxCollider)a, (CustomPhysics.v_1_1.Colliders.BoxCollider)b, out manifold);
            }
            else if (a is CustomPhysics.v_1_1.Colliders.SphereCollider && b is CustomPhysics.v_1_1.Colliders.BoxCollider)
            {
                return SphereVsBox((CustomPhysics.v_1_1.Colliders.SphereCollider)a, (CustomPhysics.v_1_1.Colliders.BoxCollider)b, out manifold);
            }
            else if (a is CustomPhysics.v_1_1.Colliders.BoxCollider && b is CustomPhysics.v_1_1.Colliders.SphereCollider)
            {
                bool result = SphereVsBox((CustomPhysics.v_1_1.Colliders.SphereCollider)b, (CustomPhysics.v_1_1.Colliders.BoxCollider)a, out manifold);
                manifold.normal = -manifold.normal; // Inverser la normale
                return result;
            }
            
            return false;
        }
        
        private static bool SphereVsSphere(CustomPhysics.v_1_1.Colliders.SphereCollider a, CustomPhysics.v_1_1.Colliders.SphereCollider b, out CollisionManifold manifold)
        {
            manifold = new CollisionManifold();
            
            Vector3 direction = b.transform.position - a.transform.position;
            float distance = direction.magnitude;
            float combinedRadius = a.Radius + b.Radius;
            
            if (distance >= combinedRadius) return false;
            
            manifold.colliderA = a;
            manifold.colliderB = b;
            manifold.normal = direction.normalized;
            manifold.penetrationDepth = combinedRadius - distance;
            manifold.contactPoint = a.transform.position + manifold.normal * a.Radius;
            
            return true;
        }
        
        private static bool BoxVsBox(CustomPhysics.v_1_1.Colliders.BoxCollider a, CustomPhysics.v_1_1.Colliders.BoxCollider b, out CollisionManifold manifold)
        {
            manifold = new CollisionManifold();
            // Implémentation SAT (Separating Axis Theorem)
            // À compléter avec l'algorithme complet
            return false;
        }
        
        private static bool SphereVsBox(CustomPhysics.v_1_1.Colliders.SphereCollider sphere, CustomPhysics.v_1_1.Colliders.BoxCollider box, out CollisionManifold manifold)
        {
            manifold = new CollisionManifold();
            
            // Trouver le point le plus proche sur la boîte
            Vector3 boxCenter = box.transform.position;
            Vector3 boxHalfSize = box.Size * 0.5f;
            
            Vector3 sphereCenter = sphere.transform.position;
            Vector3 closestPoint = sphereCenter;
            
            // Clamper le point de la sphère aux limites de la boîte
            closestPoint.x = Mathf.Clamp(closestPoint.x, boxCenter.x - boxHalfSize.x, boxCenter.x + boxHalfSize.x);
            closestPoint.y = Mathf.Clamp(closestPoint.y, boxCenter.y - boxHalfSize.y, boxCenter.y + boxHalfSize.y);
            closestPoint.z = Mathf.Clamp(closestPoint.z, boxCenter.z - boxHalfSize.z, boxCenter.z + boxHalfSize.z);
            
            // Vérifier la collision
            Vector3 direction = sphereCenter - closestPoint;
            float distance = direction.magnitude;
            
            if (distance >= sphere.Radius) return false;
            
            manifold.colliderA = sphere;
            manifold.colliderB = box;
            manifold.normal = direction.normalized;
            manifold.penetrationDepth = sphere.Radius - distance;
            manifold.contactPoint = closestPoint;
            
            return true;
        }
    }
    
    public struct CollisionManifold
    {
        public CustomCollider colliderA;
        public CustomCollider colliderB;
        public Vector3 normal;
        public float penetrationDepth;
        public Vector3 contactPoint;
    }
}