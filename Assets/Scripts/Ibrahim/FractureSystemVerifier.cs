using UnityEngine;

namespace S7an
{
    /// <summary>
    /// Quick verification and testing script for the thin plate fracture system
    /// Add this to any GameObject to get runtime information and controls
    /// </summary>
    public class FractureSystemVerifier : MonoBehaviour
    {
        [Header("System References")]
        public ThinPlateFracture plateSystem;
        
        [Header("Debug Info")]
        public bool showDebugInfo = true;
        public bool showConstraintCount = true;
        public bool showEnergyLevels = true;
        
        [Header("Testing Controls")]
        [Tooltip("Apply external force to fragments")]
        public float testForceStrength = 100f;
        
        void Start()
        {
            // Auto-find the plate system if not assigned
            if (plateSystem == null)
            {
                plateSystem = FindObjectOfType<ThinPlateFracture>();
            }
            
            if (plateSystem == null)
            {
                Debug.LogError("ThinPlateFracture system not found! Make sure it's in the scene.");
                return;
            }
            
            Debug.Log("✅ Thin Plate Fracture System found and verified!");
            Debug.Log($"📊 System configured for {plateSystem.fragmentsX}x{plateSystem.fragmentsZ} fragments");
            Debug.Log($"⚡ Energy transfer coefficient (alpha): {plateSystem.alpha}");
        }
        
        void Update()
        {
            if (plateSystem == null) return;
            
            // Runtime controls
            HandleKeyboardInput();
        }
        
        void HandleKeyboardInput()
        {
            // Start/Stop simulation
            if (Input.GetKeyDown(KeyCode.Space))
            {
                plateSystem.StartSimulation();
                Debug.Log("🚀 Simulation started!");
            }
            
            // Reset simulation  
            if (Input.GetKeyDown(KeyCode.R))
            {
                plateSystem.ResetSimulation();
                Debug.Log("🔄 Simulation reset!");
            }
            
            // Apply test force
            if (Input.GetKeyDown(KeyCode.F))
            {
                ApplyTestForce();
            }
            
            // Cycle alpha values
            if (Input.GetKeyDown(KeyCode.Alpha1)) SetAlpha(0f);
            if (Input.GetKeyDown(KeyCode.Alpha2)) SetAlpha(0.5f);
            if (Input.GetKeyDown(KeyCode.Alpha3)) SetAlpha(1f);
            if (Input.GetKeyDown(KeyCode.Alpha4)) SetAlpha(1.5f);
            if (Input.GetKeyDown(KeyCode.Alpha5)) SetAlpha(2f);
        }
        
        void ApplyTestForce()
        {
            // Find all fragments and apply random forces
            RigidFragment[] fragments = FindObjectsOfType<RigidFragment>();
            
            if (fragments.Length == 0)
            {
                Debug.LogWarning("No fragments found to apply force to!");
                return;
            }
            
            // Apply force to random fragment
            RigidFragment randomFragment = fragments[Random.Range(0, fragments.Length)];
            Vector3 randomForce = Random.onUnitSphere * testForceStrength;
            randomFragment.AddForce(randomForce);
            
            Debug.Log($"💥 Applied test force {randomForce.magnitude:F1}N to {randomFragment.name}");
        }
        
        void SetAlpha(float newAlpha)
        {
            plateSystem.alpha = newAlpha;
            Debug.Log($"⚡ Energy transfer coefficient set to: {newAlpha}");
        }
        
        void OnGUI()
        {
            if (!showDebugInfo) return;
            
            GUILayout.BeginArea(new Rect(10, 10, 400, 300));
            GUILayout.Label("=== THIN PLATE FRACTURE SYSTEM ===");
            
            if (plateSystem != null)
            {
                GUILayout.Label($"Fragments: {plateSystem.fragmentsX}x{plateSystem.fragmentsZ}");
                GUILayout.Label($"Alpha (Energy Transfer): {plateSystem.alpha:F1}");
                GUILayout.Label($"Drop Height: {plateSystem.dropHeight:F1}m");
                GUILayout.Label($"Sphere Radius: {plateSystem.sphereRadius:F1}m");
            }
            
            GUILayout.Space(10);
            GUILayout.Label("=== CONTROLS ===");
            GUILayout.Label("SPACE - Start Simulation");
            GUILayout.Label("R - Reset Simulation");  
            GUILayout.Label("F - Apply Test Force");
            GUILayout.Label("1-5 - Set Alpha (0, 0.5, 1, 1.5, 2)");
            
            GUILayout.Space(10);
            
            // Runtime fragment count
            if (showConstraintCount)
            {
                RigidFragment[] fragments = FindObjectsOfType<RigidFragment>();
                GUILayout.Label($"Active Fragments: {fragments.Length}");
            }
            
            // Energy information
            if (showEnergyLevels)
            {
                ThinPlateFracture[] systems = FindObjectsOfType<ThinPlateFracture>();
                if (systems.Length > 0)
                {
                    // This would require exposing constraint list, simplified for now
                    GUILayout.Label("Energy System: Active");
                }
            }
            
            GUILayout.EndArea();
        }
        
        /// <summary>
        /// Verify all components are working correctly
        /// </summary>
        [ContextMenu("Run System Verification")]
        public void VerifySystem()
        {
            Debug.Log("🔍 Running system verification...");
            
            // Check main system
            if (plateSystem == null)
            {
                Debug.LogError("❌ ThinPlateFracture system not found!");
                return;
            }
            Debug.Log("✅ ThinPlateFracture system found");
            
            // Check fragments
            RigidFragment[] fragments = FindObjectsOfType<RigidFragment>();
            Debug.Log($"✅ Found {fragments.Length} RigidFragment components");
            
            // Check for proper script references
            bool hasValidMass = true;
            foreach (var fragment in fragments)
            {
                if (fragment.mass <= 0)
                {
                    hasValidMass = false;
                    break;
                }
            }
            
            if (hasValidMass)
                Debug.Log("✅ All fragments have valid mass");
            else
                Debug.LogWarning("⚠️ Some fragments have invalid mass");
            
            // Check physics parameters
            if (plateSystem.gravity.magnitude > 0)
                Debug.Log("✅ Gravity is configured");
            else
                Debug.LogWarning("⚠️ Gravity is not configured");
                
            if (plateSystem.alpha >= 0 && plateSystem.alpha <= 2)
                Debug.Log($"✅ Alpha parameter is valid: {plateSystem.alpha}");
            else
                Debug.LogWarning($"⚠️ Alpha parameter may be extreme: {plateSystem.alpha}");
            
            Debug.Log("🎯 System verification complete!");
        }
    }
}