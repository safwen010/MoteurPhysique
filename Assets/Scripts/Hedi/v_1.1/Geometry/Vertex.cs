using UnityEngine;

namespace CustomPhysics.v_1_1.Geometry
{
    [System.Serializable]
    public struct Vertex
    {
        public Vector3 position;
        public Vector3 normal;
        public Vector2 uv;
        
        public Vertex(Vector3 position)
        {
            this.position = position;
            this.normal = Vector3.zero;
            this.uv = Vector2.zero;
        }
        
        public Vertex(Vector3 position, Vector3 normal, Vector2 uv)
        {
            this.position = position;
            this.normal = normal;
            this.uv = uv;
        }
    }
}