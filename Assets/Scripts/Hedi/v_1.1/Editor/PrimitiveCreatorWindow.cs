#if UNITY_EDITOR
using UnityEditor;
using UnityEngine;
using CustomPhysics.v_1_1.Objects;
using CustomPhysics.v_1_1.Colliders;

namespace CustomPhysics.v_1_1.Editor
{
    public class PrimitiveCreatorWindow : EditorWindow
    {
        private PrimitiveType primitiveType = PrimitiveType.Cube;
        private float size = 1.0f;
        private float mass = 1.0f;
        private Material customMaterial;
        
        [MenuItem("Custom Physics/Create Primitive")]
        public static void ShowWindow()
        {
            GetWindow<PrimitiveCreatorWindow>("Physics Primitive Creator");
        }
        
        void OnGUI()
        {
            GUILayout.Label("Create Custom Physics Object", EditorStyles.boldLabel);
            
            // Sélection du type
            primitiveType = (PrimitiveType)EditorGUILayout.EnumPopup("Primitive Type", primitiveType);
            
            // Paramètres de taille
            size = EditorGUILayout.FloatField("Size", size);
            mass = EditorGUILayout.FloatField("Mass", mass);
            customMaterial = (Material)EditorGUILayout.ObjectField("Material", customMaterial, typeof(Material), false);
            
            GUILayout.Space(10);
            
            if (GUILayout.Button("Create Object"))
            {
                CreatePrimitiveObject();
            }
            
            // Aide
            GUILayout.Space(10);
            EditorGUILayout.HelpBox("This will create a custom physics object with all necessary components.", MessageType.Info);
        }
        
        private void CreatePrimitiveObject()
        {
            GameObject newObject = new GameObject("Custom_" + primitiveType);
            
            // Ajouter le MeshFilter et MeshRenderer standard d'abord
            MeshFilter meshFilter = newObject.AddComponent<MeshFilter>();
            MeshRenderer meshRenderer = newObject.AddComponent<MeshRenderer>();
            
            // Générer le mesh
            UnityEngine.Mesh mesh = null;
            switch (primitiveType)
            {
                case PrimitiveType.Cube:
                    mesh = CreateCubeMesh(size);
                    break;
                case PrimitiveType.Sphere:
                    mesh = CreateSphereMesh(size * 0.5f, 16);
                    break;
            }
            
            if (mesh != null)
            {
                meshFilter.mesh = mesh;
            }
            
            // Configurer le matériau
            if (customMaterial != null)
            {
                meshRenderer.material = customMaterial;
            }
            else
            {
                // Créer un matériau par défaut
                meshRenderer.material = CreateDefaultMaterial();
            }
            
            // Ajouter le CustomMeshRenderer
            var customMeshRenderer = newObject.AddComponent<CustomMeshRenderer>();
            customMeshRenderer.useCustomMesh = false; // On utilise le mesh standard
            
            // Ajouter le rigidbody personnalisé
            var rigidbody = newObject.AddComponent<CustomPhysics.v_1_1.Objects.CustomRigidBody>();
            rigidbody.mass = mass;
            
            // Ajouter le collider approprié
            CustomCollider collider = null;
            switch (primitiveType)
            {
                case PrimitiveType.Cube:
                    var boxCollider = newObject.AddComponent<CustomPhysics.v_1_1.Colliders.BoxCollider>();
                    boxCollider.size = new Vector3(size, size, size);
                    collider = boxCollider;
                    break;
                case PrimitiveType.Sphere:
                    var sphereCollider = newObject.AddComponent<CustomPhysics.v_1_1.Colliders.SphereCollider>();
                    sphereCollider.radius = size * 0.5f;
                    collider = sphereCollider;
                    break;
            }
            
            // Positionner l'objet devant la caméra
            PlaceObjectInFrontOfCamera(newObject);
            
            // Sélectionner le nouvel objet
            Selection.activeGameObject = newObject;
            SceneView.FrameLastActiveSceneView();
            
            Debug.Log("Created custom physics object: " + newObject.name);
        }
        
        private UnityEngine.Mesh CreateCubeMesh(float size)
        {
            UnityEngine.Mesh mesh = new UnityEngine.Mesh();
            mesh.name = "CustomCube";
            
            float halfSize = size * 0.5f;
            
            // Vertices
            Vector3[] vertices = new Vector3[8]
            {
                new Vector3(-halfSize, -halfSize, -halfSize),
                new Vector3(halfSize, -halfSize, -halfSize),
                new Vector3(halfSize, halfSize, -halfSize),
                new Vector3(-halfSize, halfSize, -halfSize),
                new Vector3(-halfSize, halfSize, halfSize),
                new Vector3(halfSize, halfSize, halfSize),
                new Vector3(halfSize, -halfSize, halfSize),
                new Vector3(-halfSize, -halfSize, halfSize)
            };
            
            // Triangles (12 triangles = 36 indices)
            int[] triangles = new int[36]
            {
                // Face avant
                0, 2, 1, 0, 3, 2,
                // Face haut
                3, 4, 2, 2, 4, 5,
                // Face droite
                1, 2, 5, 1, 5, 6,
                // Face gauche
                0, 7, 4, 0, 4, 3,
                // Face bas
                0, 6, 7, 0, 1, 6,
                // Face arrière
                4, 7, 6, 4, 6, 5
            };
            
            // Normales
            Vector3[] normals = new Vector3[8];
            for (int i = 0; i < 8; i++)
            {
                normals[i] = vertices[i].normalized;
            }
            
            mesh.vertices = vertices;
            mesh.triangles = triangles;
            mesh.normals = normals;
            
            mesh.RecalculateBounds();
            
            return mesh;
        }
        
        private UnityEngine.Mesh CreateSphereMesh(float radius, int segments)
        {
            UnityEngine.Mesh mesh = new UnityEngine.Mesh();
            mesh.name = "CustomSphere";
            
            int verticalSegments = Mathf.Max(3, segments);
            int horizontalSegments = Mathf.Max(4, segments * 2);
            
            Vector3[] vertices = new Vector3[(verticalSegments + 1) * (horizontalSegments + 1)];
            Vector2[] uv = new Vector2[vertices.Length];
            int[] triangles = new int[verticalSegments * horizontalSegments * 6];
            
            // Génération des vertices
            int vertexIndex = 0;
            for (int v = 0; v <= verticalSegments; v++)
            {
                float verticalAngle = v * Mathf.PI / verticalSegments;
                float y = Mathf.Cos(verticalAngle) * radius;
                float horizontalRadius = Mathf.Sin(verticalAngle) * radius;
                
                for (int h = 0; h <= horizontalSegments; h++)
                {
                    float horizontalAngle = h * 2 * Mathf.PI / horizontalSegments;
                    float x = Mathf.Cos(horizontalAngle) * horizontalRadius;
                    float z = Mathf.Sin(horizontalAngle) * horizontalRadius;
                    
                    vertices[vertexIndex] = new Vector3(x, y, z);
                    uv[vertexIndex] = new Vector2((float)h / horizontalSegments, (float)v / verticalSegments);
                    vertexIndex++;
                }
            }
            
            // Génération des triangles
            int triangleIndex = 0;
            for (int v = 0; v < verticalSegments; v++)
            {
                for (int h = 0; h < horizontalSegments; h++)
                {
                    int current = v * (horizontalSegments + 1) + h;
                    int next = current + horizontalSegments + 1;
                    
                    triangles[triangleIndex++] = current;
                    triangles[triangleIndex++] = next + 1;
                    triangles[triangleIndex++] = current + 1;
                    
                    triangles[triangleIndex++] = current;
                    triangles[triangleIndex++] = next;
                    triangles[triangleIndex++] = next + 1;
                }
            }
            
            mesh.vertices = vertices;
            mesh.triangles = triangles;
            mesh.uv = uv;
            mesh.RecalculateNormals();
            mesh.RecalculateBounds();
            
            return mesh;
        }
        
        private Material CreateDefaultMaterial()
        {
            // Créer un matériau par défaut avec une couleur
            Shader defaultShader = Shader.Find("Standard");
            Material material = new Material(defaultShader);
            material.color = GetDefaultColorForPrimitive(primitiveType);
            material.name = "Default_" + primitiveType + "_Material";
            return material;
        }
        
        private Color GetDefaultColorForPrimitive(PrimitiveType type)
        {
            switch (type)
            {
                case PrimitiveType.Cube:
                    return Color.red;
                case PrimitiveType.Sphere:
                    return Color.blue;
                default:
                    return Color.white;
            }
        }
        
        private void PlaceObjectInFrontOfCamera(GameObject obj)
        {
            SceneView sceneView = SceneView.lastActiveSceneView;
            if (sceneView != null && sceneView.camera != null)
            {
                Camera camera = sceneView.camera;
                Vector3 position = camera.transform.position + camera.transform.forward * 5f;
                obj.transform.position = position;
            }
            else
            {
                obj.transform.position = Vector3.zero;
            }
        }
        
        public enum PrimitiveType
        {
            Cube,
            Sphere
        }
    }
}
#endif