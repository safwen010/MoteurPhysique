#if UNITY_EDITOR
using UnityEditor;
using UnityEngine;
using CustomPhysics.v_1_1.Core;

namespace CustomPhysics.v_1_1.Editor
{
    [CustomEditor(typeof(PhysicsSettings))]
    public class PhysicsSettingsEditor : UnityEditor.Editor
    {
        private PhysicsSettings settings;
        
        public override void OnInspectorGUI()
        {
            settings = (PhysicsSettings)target;
            
            EditorGUILayout.LabelField("Custom Physics Settings", EditorStyles.boldLabel);
            EditorGUILayout.Space();
            
            // World Settings
            DrawWorldSettings();
            EditorGUILayout.Space();
            
            // Collision Settings
            DrawCollisionSettings();
            EditorGUILayout.Space();
            
            // Material Settings
            DrawMaterialSettings();
            EditorGUILayout.Space();
            
            // Debug Settings
            DrawDebugSettings();
            EditorGUILayout.Space();
            
            // Boutons
            DrawActionButtons();
            
            // Sauvegarder les modifications
            if (GUI.changed)
            {
                EditorUtility.SetDirty(settings);
            }
        }
        
        private void DrawWorldSettings()
        {
            EditorGUILayout.LabelField("World Settings", EditorStyles.boldLabel);
            EditorGUILayout.BeginVertical("box");
            
            settings.gravity = EditorGUILayout.Vector3Field("Gravity", settings.gravity);
            settings.timeScale = EditorGUILayout.Slider("Time Scale", settings.timeScale, 0.1f, 2.0f);
            settings.solverIterations = EditorGUILayout.IntSlider("Solver Iterations", settings.solverIterations, 1, 20);
            
            EditorGUILayout.EndVertical();
        }
        
        private void DrawCollisionSettings()
        {
            EditorGUILayout.LabelField("Collision Settings", EditorStyles.boldLabel);
            EditorGUILayout.BeginVertical("box");
            
            settings.collisionTolerance = EditorGUILayout.FloatField("Collision Tolerance", settings.collisionTolerance);
            settings.penetrationSlop = EditorGUILayout.FloatField("Penetration Slop", settings.penetrationSlop);
            settings.biasFactor = EditorGUILayout.Slider("Bias Factor", settings.biasFactor, 0.0f, 1.0f);
            
            EditorGUILayout.EndVertical();
        }
        
        private void DrawMaterialSettings()
        {
            EditorGUILayout.LabelField("Material Settings", EditorStyles.boldLabel);
            EditorGUILayout.BeginVertical("box");
            
            settings.defaultFriction = EditorGUILayout.Slider("Default Friction", settings.defaultFriction, 0.0f, 1.0f);
            settings.defaultRestitution = EditorGUILayout.Slider("Default Restitution", settings.defaultRestitution, 0.0f, 1.0f);
            
            EditorGUILayout.EndVertical();
        }
        
        private void DrawDebugSettings()
        {
            EditorGUILayout.LabelField("Debug Settings", EditorStyles.boldLabel);
            EditorGUILayout.BeginVertical("box");
            
            settings.drawGizmos = EditorGUILayout.Toggle("Draw Gizmos", settings.drawGizmos);
            settings.logCollisions = EditorGUILayout.Toggle("Log Collisions", settings.logCollisions);
            settings.gizmoColor = EditorGUILayout.ColorField("Gizmo Color", settings.gizmoColor);
            
            EditorGUILayout.EndVertical();
        }
        
        private void DrawActionButtons()
        {
            EditorGUILayout.BeginHorizontal();
            
            if (GUILayout.Button("Reset to Default"))
            {
                ResetToDefault();
            }
            
            if (GUILayout.Button("Apply to Scene"))
            {
                ApplyToScene();
            }
            
            EditorGUILayout.EndHorizontal();
        }
        
        private void ResetToDefault()
        {
            settings.gravity = new Vector3(0, -9.81f, 0);
            settings.timeScale = 1.0f;
            settings.solverIterations = 10;
            settings.collisionTolerance = 0.01f;
            settings.penetrationSlop = 0.01f;
            settings.biasFactor = 0.2f;
            settings.defaultFriction = 0.2f;
            settings.defaultRestitution = 0.1f;
            settings.drawGizmos = true;
            settings.logCollisions = false;
            settings.gizmoColor = Color.green;
        }
        
        private void ApplyToScene()
        {
            var physicsEngine = FindFirstObjectByType<CustomPhysicsEngine>();
            if (physicsEngine != null)
            {
                physicsEngine.ApplySettings(settings);
                Debug.Log("Physics settings applied to scene");
            }
        }
    }
}
#endif