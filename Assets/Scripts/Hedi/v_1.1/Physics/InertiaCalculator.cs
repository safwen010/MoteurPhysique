using UnityEngine;
using CustomPhysics.v_1_1.Objects;
using CustomPhysics.v_1_1.Colliders;

namespace CustomPhysics.v_1_1.Physics
{
    public static class InertiaCalculator
    {
        public static Matrix4x4 CalculateInverseInertiaTensor(float mass, CustomPhysics.v_1_1.Objects.CustomCollider collider)
        {
            Matrix4x4 inertiaTensor = Matrix4x4.identity;
            
            switch (collider.type)
            {
                case CustomPhysics.v_1_1.Objects.CustomCollider.ColliderType.Sphere:
                    inertiaTensor = CalculateSphereInertia(mass, (CustomPhysics.v_1_1.Colliders.SphereCollider)collider);
                    break;
                case CustomPhysics.v_1_1.Objects.CustomCollider.ColliderType.Box:
                    inertiaTensor = CalculateBoxInertia(mass, (CustomPhysics.v_1_1.Colliders.BoxCollider)collider);
                    break;
                default:
                    inertiaTensor = CalculateDefaultInertia(mass);
                    break;
            }
            
            return inertiaTensor.inverse;
        }
        
        private static Matrix4x4 CalculateSphereInertia(float mass, CustomPhysics.v_1_1.Colliders.SphereCollider sphere)
        {
            float radius = sphere.Radius;
            float diag = (2.0f / 5.0f) * mass * radius * radius;
            
            Matrix4x4 inertia = Matrix4x4.identity;
            inertia.m00 = diag;
            inertia.m11 = diag;
            inertia.m22 = diag;
            
            return inertia;
        }
        
        private static Matrix4x4 CalculateBoxInertia(float mass, CustomPhysics.v_1_1.Colliders.BoxCollider box)
        {
            Vector3 size = box.Size;
            float x2 = size.x * size.x;
            float y2 = size.y * size.y;
            float z2 = size.z * size.z;
            
            Matrix4x4 inertia = Matrix4x4.identity;
            inertia.m00 = mass * (y2 + z2) / 12f;
            inertia.m11 = mass * (x2 + z2) / 12f;
            inertia.m22 = mass * (x2 + y2) / 12f;
            
            return inertia;
        }
        
        private static Matrix4x4 CalculateDefaultInertia(float mass)
        {
            // Inertie par défaut pour un cube unitaire
            Matrix4x4 inertia = Matrix4x4.identity;
            inertia.m00 = mass * 0.166f;
            inertia.m11 = mass * 0.166f;
            inertia.m22 = mass * 0.166f;
            
            return inertia;
        }
    }
}