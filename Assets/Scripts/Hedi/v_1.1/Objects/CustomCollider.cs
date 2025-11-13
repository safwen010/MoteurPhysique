using UnityEngine;
using CustomPhysics.v_1_1.Core;
using CustomPhysics.v_1_1.Physics;
using CustomPhysics.v_1_1.Geometry;

namespace CustomPhysics.v_1_1.Objects
{
    public abstract class CustomCollider : MonoBehaviour
    {
        public enum ColliderType { Sphere, Box, Mesh }
        
        [Header("Collider Settings")]
        public ColliderType type;
        public bool isTrigger = false;
        public PhysicsMaterial physicsMaterial;
        
        public CustomPhysics.v_1_1.Geometry.Bounds Bounds { get; protected set; }
        public Vector3 Center => transform.position + centerOffset;
        
        [SerializeField] protected Vector3 centerOffset = Vector3.zero;
        
        protected CustomRigidBody rigidBody;
        
        public CustomRigidBody RigidBody => rigidBody;
        
        void Awake()
        {
            rigidBody = GetComponent<CustomRigidBody>();
            CalculateBounds();
            
            // Enregistrement dans le moteur physique
            CustomPhysicsEngine.Instance.RegisterCollider(this);
        }
        
        void OnDestroy()
        {
            if (CustomPhysicsEngine.Instance != null)
                CustomPhysicsEngine.Instance.UnregisterCollider(this);
        }
        
        void Update()
        {
            if (transform.hasChanged)
            {
                CalculateBounds();
                transform.hasChanged = false;
            }
        }
        
        public abstract void CalculateBounds();
        public abstract bool CheckCollision(CustomCollider other, out CollisionManifold manifold);
        
        public virtual void DrawGizmos()
        {
            Gizmos.color = isTrigger ? Color.yellow : Color.green;
            Gizmos.DrawWireCube(Bounds.center, Bounds.size);
        }
        
        void OnDrawGizmosSelected()
        {
            DrawGizmos();
        }
    }
    
    [System.Serializable]
    public class PhysicsMaterial
    {
        public float friction = 0.2f;
        public float restitution = 0.1f;
    }
}