using UnityEngine;
using System.Collections.Generic;

namespace ChainSim
{
    /// <summary>
    /// Generates a chain of rectangular links (Fragments) between two anchor cubes.
    /// No PhysX is used. Fragments are child objects with mesh renderers for visibility only.
    /// </summary>
    public class ChainGenerator : MonoBehaviour
    {
        [Header("Chain Layout")]
        public int linkCount = 16;
        public Vector3 linkSize = new Vector3(0.2f, 0.4f, 0.2f); // width, height, depth
        public float linkMass = 0.5f;
        public float gap = 0.02f; // small spacing between link centers

        [Header("Anchors")]
        public Vector3 anchorSize = new Vector3(0.5f, 0.5f, 0.5f);
        public float anchorMass = 2f;

        public Fragment topAnchor { get; private set; }
        public Fragment bottomAnchor { get; private set; }
        public List<Fragment> links = new List<Fragment>();

        public void Clear()
        {
            var toDestroy = new List<GameObject>();
            foreach (Transform c in transform) toDestroy.Add(c.gameObject);
#if UNITY_EDITOR
            for (int i = 0; i < toDestroy.Count; i++) Object.DestroyImmediate(toDestroy[i]);
#else
            for (int i = 0; i < toDestroy.Count; i++) Destroy(toDestroy[i]);
#endif
            links.Clear(); topAnchor = bottomAnchor = null;
        }

        public void Generate()
        {
            Clear();

            // Create anchors
            topAnchor = CreateBox("TopAnchor", anchorSize, anchorMass, Color.gray);
            bottomAnchor = CreateBox("BottomAnchor", anchorSize, anchorMass, Color.gray);

            // Arrange vertically along -Y
            float totalLen = linkCount * (linkSize.y + gap);
            Vector3 start = new Vector3(0, totalLen * 0.5f + anchorSize.y * 0.5f, 0);
            topAnchor.transform.localPosition = start;
            bottomAnchor.transform.localPosition = new Vector3(0, -totalLen * 0.5f - anchorSize.y * 0.5f, 0);

            // Create links spaced between anchors
            Vector3 cur = start - new Vector3(0, anchorSize.y * 0.5f + (linkSize.y * 0.5f), 0);
            for (int i = 0; i < linkCount; i++)
            {
                var link = CreateBox($"Link_{i}", linkSize, linkMass, Color.Lerp(Color.cyan, Color.blue, (float)i / Mathf.Max(1, linkCount-1)));
                link.transform.localPosition = cur;
                links.Add(link);
                cur += new Vector3(0, -(linkSize.y + gap), 0);
            }

            // Slight rotation to avoid perfect alignment
            transform.rotation = Quaternion.identity;
        }

        private Fragment CreateBox(string name, Vector3 size, float mass, Color color)
        {
            GameObject go = GameObject.CreatePrimitive(PrimitiveType.Cube);
            go.name = name;
            go.transform.SetParent(transform, false);
            go.transform.localScale = size;
            var col = go.GetComponent<Collider>(); if (col) DestroyImmediate(col);

            var frag = go.AddComponent<Fragment>();
            frag.mass = mass;
            frag.size = size;

            var mr = go.GetComponent<MeshRenderer>();
            if (mr)
            {
                // Create material with a shader that exists in the project (fallback for URP)
                Shader sh = Shader.Find("Standard");
                if (sh == null) sh = Shader.Find("Universal Render Pipeline/Lit");
                if (sh == null) sh = Shader.Find("Sprites/Default");
                Material mat = null;
                if (sh != null) mat = new Material(sh);
                else mat = new Material(Shader.Find("Hidden/InternalErrorShader"));

                // Set color using property name used by common shaders
                if (mat.HasProperty("_Color")) mat.SetColor("_Color", color);
                else if (mat.HasProperty("color")) mat.SetColor("color", color);

                // Assign instance material so each link can have its own color
                mr.material = mat;
            }
            return frag;
        }

        private void OnDrawGizmosSelected()
        {
            Gizmos.color = Color.yellow;
            Gizmos.matrix = transform.localToWorldMatrix;
            Gizmos.DrawWireCube(Vector3.zero, new Vector3(anchorSize.x, linkCount * (linkSize.y + gap) + anchorSize.y * 2f, anchorSize.z));
        }
    }
}
