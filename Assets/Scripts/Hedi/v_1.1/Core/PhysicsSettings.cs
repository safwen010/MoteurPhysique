using UnityEngine;

namespace CustomPhysics.v_1_1.Core
{
    [CreateAssetMenu(fileName = "PhysicsSettings", menuName = "Custom Physics/Physics Settings")]
    public class PhysicsSettings : ScriptableObject
    {
        [Header("World Settings")]
        public Vector3 gravity = new Vector3(0, -9.81f, 0);
        public float timeScale = 1.0f;
        public int solverIterations = 10;
        
        [Header("Collision Settings")]
        public float collisionTolerance = 0.01f;
        public float penetrationSlop = 0.01f;
        public float biasFactor = 0.2f;
        
        [Header("Physics Materials")]
        public float defaultFriction = 0.2f;
        public float defaultRestitution = 0.1f;
        
        [Header("Debug")]
        public bool drawGizmos = true;
        public bool logCollisions = false;
        public Color gizmoColor = Color.green;
    }
}