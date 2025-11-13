using UnityEngine;
using CustomPhysics.v_1_1.Objects;
using CustomPhysics.v_1_1.Core;

namespace CustomPhysics.v_1_1.Physics
{
    public static class CollisionResolver
    {
        public static void Resolve(CollisionManifold manifold, PhysicsSettings settings)
        {
            if (manifold.colliderA.isTrigger || manifold.colliderB.isTrigger)
                return;
                
            CustomRigidBody bodyA = manifold.colliderA.RigidBody;
            CustomRigidBody bodyB = manifold.colliderB.RigidBody;
            
            if (bodyA == null && bodyB == null) return;
            
            // Résolution de la pénétration
            ResolvePenetration(manifold, bodyA, bodyB);
            
            // Résolution de la vitesse
            ResolveVelocity(manifold, bodyA, bodyB, settings);
        }
        
        private static void ResolvePenetration(CollisionManifold manifold, CustomRigidBody bodyA, CustomRigidBody bodyB)
        {
            float totalInverseMass = bodyA?.InverseMass ?? 0 + bodyB?.InverseMass ?? 0;
            if (totalInverseMass <= 0) return;
            
            Vector3 correction = manifold.normal * (manifold.penetrationDepth / totalInverseMass);
            
            if (bodyA != null && !bodyA.IsKinematic)
            {
                bodyA.transform.position -= correction * bodyA.InverseMass;
            }
            
            if (bodyB != null && !bodyB.IsKinematic)
            {
                bodyB.transform.position += correction * bodyB.InverseMass;
            }
        }
        
        private static void ResolveVelocity(CollisionManifold manifold, CustomRigidBody bodyA, CustomRigidBody bodyB, PhysicsSettings settings)
        {
            if (bodyA == null || bodyB == null) return;
            if (bodyA.IsKinematic && bodyB.IsKinematic) return;
            
            Vector3 relativeVelocity = bodyB.velocity - bodyA.velocity;
            float velocityAlongNormal = Vector3.Dot(relativeVelocity, manifold.normal);
            
            // Les objets s'éloignent déjà
            if (velocityAlongNormal > 0) return;
            
            // Coefficient de restitution
            float restitution = Mathf.Min(
                bodyA?.restitution ?? settings.defaultRestitution,
                bodyB?.restitution ?? settings.defaultRestitution
            );
            
            float totalInverseMass = bodyA.InverseMass + bodyB.InverseMass;
            float j = -(1 + restitution) * velocityAlongNormal / totalInverseMass;
            
            Vector3 impulse = manifold.normal * j;
            
            if (!bodyA.IsKinematic)
            {
                bodyA.velocity -= impulse * bodyA.InverseMass;
            }
            
            if (!bodyB.IsKinematic)
            {
                bodyB.velocity += impulse * bodyB.InverseMass;
            }
        }
    }
}