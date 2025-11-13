using UnityEngine;

namespace CustomPhysics.v_1_1.Objects
{
    public static class PrimitiveGenerator
    {
        public static UnityEngine.Mesh CreateCube(float size)
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
        
        public static UnityEngine.Mesh CreateSphere(float radius, int segments)
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
    }
}