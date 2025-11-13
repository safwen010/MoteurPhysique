using UnityEngine;
using System.Collections.Generic;

namespace CustomPhysics.v_1_1.Geometry
{
    [System.Serializable]
    public class Mesh
    {
        public List<Vertex> vertices = new List<Vertex>();
        public List<int> triangles = new List<int>();
        public List<Vector3> normals = new List<Vector3>();
        public List<Vector2> uv = new List<Vector2>();
        
        public Bounds bounds;
        
        public void RecalculateBounds()
        {
            if (vertices.Count == 0)
            {
                bounds = new Bounds(Vector3.zero, Vector3.zero);
                return;
            }
            
            Vector3 min = vertices[0].position;
            Vector3 max = vertices[0].position;
            
            foreach (var vertex in vertices)
            {
                min = Vector3.Min(min, vertex.position);
                max = Vector3.Max(max, vertex.position);
            }
            
            bounds = new Bounds((min + max) * 0.5f, max - min);
        }
        
        public void RecalculateNormals()
        {
            normals.Clear();
            normals.Capacity = vertices.Count;
            
            for (int i = 0; i < vertices.Count; i++)
            {
                normals.Add(Vector3.zero);
            }
            
            for (int i = 0; i < triangles.Count; i += 3)
            {
                int i1 = triangles[i];
                int i2 = triangles[i + 1];
                int i3 = triangles[i + 2];
                
                Vector3 v1 = vertices[i1].position;
                Vector3 v2 = vertices[i2].position;
                Vector3 v3 = vertices[i3].position;
                
                Vector3 normal = Vector3.Cross(v2 - v1, v3 - v1).normalized;
                
                normals[i1] += normal;
                normals[i2] += normal;
                normals[i3] += normal;
            }
            
            for (int i = 0; i < normals.Count; i++)
            {
                normals[i] = normals[i].normalized;
            }
        }
    }
    
}