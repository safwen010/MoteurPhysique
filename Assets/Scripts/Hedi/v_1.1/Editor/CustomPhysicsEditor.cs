#if UNITY_EDITOR
using UnityEditor;
using UnityEngine;
using CustomPhysics.v_1_1.Core;

namespace CustomPhysics.v_1_1.Editor
{
    [CustomEditor(typeof(CustomPhysicsEngine))]
    public class CustomPhysicsEditor : UnityEditor.Editor
    {
        private CustomPhysics.v_1_1.Core.CustomPhysicsEngine physicsEngine;
        
        public override void OnInspectorGUI()
        {
            DrawDefaultInspector();
            
            physicsEngine = (CustomPhysicsEngine)target;
            
            GUILayout.Space(10);
            
            // Section statistiques
            EditorGUILayout.LabelField("Physics Statistics", EditorStyles.boldLabel);
            EditorGUILayout.BeginVertical("box");
            
            EditorGUILayout.LabelField("RigidBodies: " + (physicsEngine.RigidBodyCount));
            EditorGUILayout.LabelField("Colliders: " + (physicsEngine.ColliderCount));
            EditorGUILayout.LabelField("Collisions: " + (physicsEngine.CollisionCount));
            
            EditorGUILayout.EndVertical();
            
            // Boutons de contrôle
            GUILayout.Space(10);
            EditorGUILayout.BeginHorizontal();
            
            if (GUILayout.Button("Reset Physics"))
            {
                physicsEngine.ResetPhysics();
            }
            
            if (GUILayout.Button("Pause/Resume"))
            {
                physicsEngine.TogglePause();
            }
            
            EditorGUILayout.EndHorizontal();
        }
    }
}
#endif