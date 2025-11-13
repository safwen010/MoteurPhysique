using UnityEngine;
using CustomPhysics.v_1_1.Core;
using CustomPhysics.v_1_1.Physics;

namespace CustomPhysics.v_1_1.Objects
{
    [RequireComponent(typeof(CustomCollider))]
    public class CustomRigidBody : MonoBehaviour
    {
        [Header("Physics Properties")]
        public float mass = 1.0f;
        public Vector3 velocity = Vector3.zero;
        public Vector3 angularVelocity = Vector3.zero;
        public bool IsKinematic = false;
        
        [Header("Material Properties")]
        public float friction = 0.2f;
        public float restitution = 0.1f;
        
        // Forces accumulées
        private Vector3 accumulatedForce = Vector3.zero;
        private Vector3 accumulatedTorque = Vector3.zero;
        
        // Propriétés dérivées
        public float InverseMass => IsKinematic ? 0 : 1.0f / mass;
        public Matrix4x4 InverseInertiaTensor { get; private set; }
        
        private CustomCollider customCollider;
        
        void Awake()
        {
            customCollider = GetComponent<CustomCollider>();
            CalculateInertiaTensor();
            
            // Enregistrement dans le moteur physique
            CustomPhysicsEngine.Instance.RegisterRigidBody(this);
        }
        
        void OnDestroy()
        {
            if (CustomPhysicsEngine.Instance != null)
                CustomPhysicsEngine.Instance.UnregisterRigidBody(this);
        }
        
        public void AddForce(Vector3 force, CustomPhysics.v_1_1.Physics.ForceMode mode = CustomPhysics.v_1_1.Physics.ForceMode.Force)
        {
            if (IsKinematic) return;
            
            switch (mode)
            {
                case CustomPhysics.v_1_1.Physics.ForceMode.Force:
                    accumulatedForce += force;
                    break;
                case CustomPhysics.v_1_1.Physics.ForceMode.Impulse:
                    velocity += force * InverseMass;
                    break;
            }
        }
        
        public void AddForceAtPosition(Vector3 force, Vector3 position)
        {
            if (IsKinematic) return;
            
            accumulatedForce += force;
            accumulatedTorque += Vector3.Cross(position - transform.position, force);
        }
        
        public void IntegrateForces(float deltaTime)
        {
            if (IsKinematic) return;
            
            // Intégration de la vitesse
            velocity += accumulatedForce * InverseMass * deltaTime;
            angularVelocity += InverseInertiaTensor.MultiplyVector(accumulatedTorque) * deltaTime;
            
            // Réinitialisation des forces
            accumulatedForce = Vector3.zero;
            accumulatedTorque = Vector3.zero;
        }
        
        public void Integrate(float deltaTime)
        {
            if (IsKinematic) return;
            
            // Intégration de la position
            transform.position += velocity * deltaTime;
            
            // Intégration de la rotation
            if (angularVelocity.sqrMagnitude > 0.001f)
            {
                Quaternion rotationDelta = Quaternion.Euler(angularVelocity * deltaTime * Mathf.Rad2Deg);
                transform.rotation = rotationDelta * transform.rotation;
            }
        }
        
        private void CalculateInertiaTensor()
        {
            if (customCollider != null)
            {
                InverseInertiaTensor = CustomPhysics.v_1_1.Physics.InertiaCalculator.CalculateInverseInertiaTensor(mass, customCollider);
            }
            else
            {
                // Tenseur d'inertie par défaut pour une sphère
                float radius = 1.0f;
                float diag = (2.0f / 5.0f) * mass * radius * radius;
                Matrix4x4 inertia = Matrix4x4.identity;
                inertia.m00 = diag;
                inertia.m11 = diag;
                inertia.m22 = diag;
                InverseInertiaTensor = inertia.inverse;
            }
        }
        
        void OnValidate()
        {
            mass = Mathf.Max(mass, 0.001f);
        }
    }
}