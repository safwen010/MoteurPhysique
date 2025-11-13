using System.Collections.Generic;
using UnityEngine;
using CustomPhysics.v_1_1.Objects;
using CustomPhysics.v_1_1.Physics;

namespace CustomPhysics.v_1_1.Core
{
    public class CustomPhysicsEngine : MonoBehaviour
    {
        [SerializeField] private PhysicsSettings settings;
        
        private List<CustomRigidBody> rigidBodies = new List<CustomRigidBody>();
        private List<CustomCollider> colliders = new List<CustomCollider>();
        private List<CollisionManifold> collisions = new List<CollisionManifold>();
        
        public static CustomPhysicsEngine Instance { get; private set; }
        
        // Propriétés pour l'éditeur (AJOUTÉ)
        public int RigidBodyCount => rigidBodies.Count;
        public int ColliderCount => colliders.Count;
        public int CollisionCount => collisions.Count;
        
        void Awake()
        {
            if (Instance == null)
            {
                Instance = this;
                InitializePhysics();
            }
        }
        
        void FixedUpdate()
        {
            if (settings == null) return;
            
            float deltaTime = Time.fixedDeltaTime * settings.timeScale;
            
            // Étape 1: Application des forces
            ApplyForces(deltaTime);
            
            // Étape 2: Détection des collisions
            DetectCollisions();
            
            // Étape 3: Résolution des collisions
            ResolveCollisions();
            
            // Étape 4: Intégration
            Integrate(deltaTime);
        }
        
        private void InitializePhysics()
        {
            Debug.Log("Custom Physics Engine Initialized");
        }
        
        private void ApplyForces(float deltaTime)
        {
            foreach (var body in rigidBodies)
            {
                if (!body.IsKinematic)
                {
                    // Gravité
                    ForceSystem.ApplyGravity(body, settings.gravity);
                    
                    // Intégration des forces
                    body.IntegrateForces(deltaTime);
                }
            }
        }
        
        private void DetectCollisions()
        {
            collisions.Clear();
            
            for (int i = 0; i < colliders.Count; i++)
            {
                for (int j = i + 1; j < colliders.Count; j++)
                {
                    if (CollisionDetection.CheckCollision(colliders[i], colliders[j], out CollisionManifold manifold))
                    {
                        collisions.Add(manifold);
                    }
                }
            }
        }
        
        private void ResolveCollisions()
        {
            foreach (var collision in collisions)
            {
                CollisionResolver.Resolve(collision, settings);
            }
        }
        
        private void Integrate(float deltaTime)
        {
            foreach (var body in rigidBodies)
            {
                if (!body.IsKinematic)
                {
                    body.Integrate(deltaTime);
                }
            }
        }
        
        public void RegisterRigidBody(CustomRigidBody body)
        {
            if (!rigidBodies.Contains(body))
                rigidBodies.Add(body);
        }
        
        public void UnregisterRigidBody(CustomRigidBody body)
        {
            rigidBodies.Remove(body);
        }
        
        public void RegisterCollider(CustomCollider collider)
        {
            if (!colliders.Contains(collider))
                colliders.Add(collider);
        }
        
        public void UnregisterCollider(CustomCollider collider)
        {
            colliders.Remove(collider);
        }

        // MÉTHODES POUR L'ÉDITEUR (AJOUTÉES)
        public void ResetPhysics()
        {
            rigidBodies.Clear();
            colliders.Clear();
            collisions.Clear();
            
            // Re-trouver tous les objets physiques dans la scène
            var bodies = FindObjectsOfType<CustomRigidBody>();
            var cols = FindObjectsOfType<CustomCollider>();
            
            rigidBodies.AddRange(bodies);
            colliders.AddRange(cols);
            
            Debug.Log($"Physics reset: {rigidBodies.Count} rigidbodies, {colliders.Count} colliders found");
        }
        
        public void TogglePause()
        {
            this.enabled = !this.enabled;
            Debug.Log($"Physics {(this.enabled ? "resumed" : "paused")}");
        }
        
        public void ApplySettings(PhysicsSettings newSettings)
        {
            if (newSettings != null)
            {
                settings = newSettings;
                Debug.Log("Physics settings applied");
            }
        }
        
        void OnDrawGizmos()
        {
            if (!settings || !settings.drawGizmos) return;
            
            Gizmos.color = settings.gizmoColor;
            
            // Dessiner les colliders
            foreach (var collider in colliders)
            {
                if (collider != null)
                    collider.DrawGizmos();
            }
            
            // Dessiner les collisions
            Gizmos.color = Color.red;
            foreach (var collision in collisions)
            {
                Gizmos.DrawSphere(collision.contactPoint, 0.1f);
                Gizmos.DrawLine(collision.contactPoint, collision.contactPoint + collision.normal * 0.5f);
            }
        }

        // Méthode pour initialiser avec des objets existants (AJOUTÉE)
        void Start()
        {
            // S'assurer que tous les objets existants sont enregistrés
            ResetPhysics();
        }
    }
}