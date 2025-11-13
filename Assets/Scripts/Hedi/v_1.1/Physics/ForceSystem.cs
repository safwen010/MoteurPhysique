using UnityEngine;
using CustomPhysics.v_1_1.Objects;

namespace CustomPhysics.v_1_1.Physics
{
    public static class ForceSystem
    {
        public static void ApplyGravity(CustomPhysics.v_1_1.Objects.CustomRigidBody body, Vector3 gravity)
        {
            if (!body.IsKinematic)
            {
                body.AddForce(gravity * body.mass, ForceMode.Force);
            }
        }
        
        public static void ApplyExplosionForce(CustomRigidBody body, float explosionForce, Vector3 explosionPosition, float explosionRadius, float upliftModifier = 0)
        {
            Vector3 explosionDir = body.transform.position - explosionPosition;
            float distance = explosionDir.magnitude;
            
            if (distance > explosionRadius) return;
            
            // Normalisation
            explosionDir.Normalize();
            
            // Force proportionnelle à la distance
            float force = explosionForce * (1 - distance / explosionRadius);
            
            // Application de la force
            body.AddForce(explosionDir * force, ForceMode.Impulse);
            
            // Force d'élévation
            if (upliftModifier != 0)
            {
                float uplift = upliftModifier * force;
                body.AddForce(Vector3.up * uplift, ForceMode.Impulse);
            }
        }
        
        public static void ApplyDamping(CustomRigidBody body, float linearDamping, float angularDamping, float deltaTime)
        {
            if (!body.IsKinematic)
            {
                body.velocity *= Mathf.Clamp01(1 - linearDamping * deltaTime);
                body.angularVelocity *= Mathf.Clamp01(1 - angularDamping * deltaTime);
            }
        }
    }
    
    public enum ForceMode
    {
        Force,
        Impulse
    }
}