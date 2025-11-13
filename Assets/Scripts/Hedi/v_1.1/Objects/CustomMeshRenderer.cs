using UnityEngine;

namespace CustomPhysics.v_1_1.Objects
{
    [RequireComponent(typeof(MeshFilter), typeof(MeshRenderer))]
    public class CustomMeshRenderer : MonoBehaviour
    {
        [Header("Rendering Settings")]
        public bool useCustomMesh = true;
        public Material customMaterial;
        
        private MeshFilter meshFilter;
        private MeshRenderer meshRenderer;
        private UnityEngine.Mesh customMesh;
        
        void Awake()
        {
            meshFilter = GetComponent<MeshFilter>();
            meshRenderer = GetComponent<MeshRenderer>();
            
            if (useCustomMesh && customMesh != null)
            {
                meshFilter.mesh = customMesh;
            }
            
            if (customMaterial != null)
            {
                meshRenderer.material = customMaterial;
            }
        }
        
        public void SetMesh(UnityEngine.Mesh mesh)
        {
            customMesh = mesh;
            if (meshFilter != null && useCustomMesh)
                meshFilter.mesh = customMesh;
        }
        
        public void SetMaterial(Material material)
        {
            customMaterial = material;
            if (meshRenderer != null)
                meshRenderer.material = customMaterial;
        }
        
        void OnValidate()
        {
            if (meshFilter == null)
                meshFilter = GetComponent<MeshFilter>();
            if (meshRenderer == null)
                meshRenderer = GetComponent<MeshRenderer>();
        }
    }
}