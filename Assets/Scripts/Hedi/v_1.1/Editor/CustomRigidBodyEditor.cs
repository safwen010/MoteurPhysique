#if UNITY_EDITOR
using UnityEditor;
using UnityEngine;
using CustomPhysics.v_1_1.Objects;

namespace CustomPhysics.v_1_1.Editor
{
    [CustomEditor(typeof(CustomRigidBody))]
    public class CustomRigidBodyEditor : UnityEditor.Editor
    {
        private CustomRigidBody rigidbody;
        private bool showAdvanced = false;
        
        public override void OnInspectorGUI()
        {
            rigidbody = (CustomRigidBody)target;
            
            EditorGUILayout.LabelField("Custom RigidBody", EditorStyles.boldLabel);
            
            // Propriétés de base
            DrawBasicProperties();
            EditorGUILayout.Space();
            
            // Propriétés avancées
            showAdvanced = EditorGUILayout.Foldout(showAdvanced, "Advanced Properties");
            if (showAdvanced)
            {
                DrawAdvancedProperties();
            }
            EditorGUILayout.Space();
            
            // Informations en lecture seule
            DrawReadOnlyInfo();
            EditorGUILayout.Space();
            
            // Boutons d'action
            DrawActionButtons();
        }
        
        private void DrawBasicProperties()
        {
            EditorGUILayout.BeginVertical("box");
            
            rigidbody.mass = EditorGUILayout.FloatField("Mass", rigidbody.mass);
            rigidbody.IsKinematic = EditorGUILayout.Toggle("Is Kinematic", rigidbody.IsKinematic);
            rigidbody.friction = EditorGUILayout.Slider("Friction", rigidbody.friction, 0.0f, 1.0f);
            rigidbody.restitution = EditorGUILayout.Slider("Restitution", rigidbody.restitution, 0.0f, 1.0f);
            
            EditorGUILayout.EndVertical();
        }
        
        private void DrawAdvancedProperties()
        {
            EditorGUILayout.BeginVertical("box");
            
            rigidbody.velocity = EditorGUILayout.Vector3Field("Velocity", rigidbody.velocity);
            rigidbody.angularVelocity = EditorGUILayout.Vector3Field("Angular Velocity", rigidbody.angularVelocity);
            
            EditorGUILayout.EndVertical();
        }
        
        private void DrawReadOnlyInfo()
        {
            EditorGUILayout.BeginVertical("box");
            EditorGUILayout.LabelField("Physics Info", EditorStyles.boldLabel);
            
            EditorGUI.BeginDisabledGroup(true);
            EditorGUILayout.FloatField("Inverse Mass", rigidbody.InverseMass);
            EditorGUILayout.Vector3Field("Position", rigidbody.transform.position);
            EditorGUILayout.Vector3Field("Rotation", rigidbody.transform.rotation.eulerAngles);
            EditorGUI.EndDisabledGroup();
            
            EditorGUILayout.EndVertical();
        }
        
        private void DrawActionButtons()
        {
            EditorGUILayout.BeginHorizontal();
            
            if (GUILayout.Button("Reset Velocity"))
            {
                rigidbody.velocity = Vector3.zero;
                rigidbody.angularVelocity = Vector3.zero;
            }
            
            if (GUILayout.Button("Apply Test Force"))
            {
                rigidbody.AddForce(Vector3.up * 10f, CustomPhysics.v_1_1.Physics.ForceMode.Impulse);
            }
            
            EditorGUILayout.EndHorizontal();
        }
    }
}
#endif