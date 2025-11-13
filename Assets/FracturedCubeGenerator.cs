using UnityEngine;

/// <summary>
/// Generates a pre-fractured cube by creating a grid of smaller fragment cubes.
/// Each fragment is an independent element with custom physics (no PhysX).
/// </summary>
public class FracturedCubeGenerator : MonoBehaviour
{
    [Header("Fracture Settings")]
    public int gridSize = 4;              // Number of fragments per axis (4 → 4×4×4 = 64 cubes)
    public float cubeSize = 0.5f;         // Size of each fragment (m)
    public float spacing = 0.0f;          // Extra gap between cubes (0 = touching, negative = overlap)
    public float fragmentMass = 1f;       // Mass of each fragment (kg)
    public float density = 1000f;         // Material density (kg/m³)
    public bool useDensity = true;        // Use density to calculate mass instead of fixed mass

    [Header("Visual Settings")]
    public Material fragmentMaterial;     // Material for fragments
    public bool randomizeColors = true;   // Randomize fragment colors

    // Storage for all fragments (for later constraint setup)
    private GameObject[,,] fragments;

    void Start()
    {
        GenerateFracturedCube();
    }

    void GenerateFracturedCube()
    {
        int total = gridSize * gridSize * gridSize;
        Debug.Log($"Generating {gridSize}×{gridSize}×{gridSize} = {total} fragments");

        fragments = new GameObject[gridSize, gridSize, gridSize];

        // Compute offset so the entire cube is centered on this GameObject
        Vector3 cubeOffset = new Vector3(
            (gridSize - 1) * 0.5f * (cubeSize + spacing),
            (gridSize - 1) * 0.5f * (cubeSize + spacing),
            (gridSize - 1) * 0.5f * (cubeSize + spacing)
        );

        // Generate each fragment
        for (int x = 0; x < gridSize; x++)
        {
            for (int y = 0; y < gridSize; y++)
            {
                for (int z = 0; z < gridSize; z++)
                {
                    // Position relative to center
                    Vector3 pos = transform.position + new Vector3(
                        x * (cubeSize + spacing),
                        y * (cubeSize + spacing),
                        z * (cubeSize + spacing)
                    ) - cubeOffset;

                    fragments[x, y, z] = CreateFragmentAt(pos, x, y, z);
                }
            }
        }

        Debug.Log("Fractured cube generation complete.");
    }

    GameObject CreateFragmentAt(Vector3 position, int x, int y, int z)
    {
        GameObject fragmentObj = GameObject.CreatePrimitive(PrimitiveType.Cube);
        fragmentObj.name = $"Fragment_{x}_{y}_{z}";
        fragmentObj.transform.position = position;
        fragmentObj.transform.localScale = Vector3.one * cubeSize;
        fragmentObj.transform.SetParent(transform, true);

        // Remove Unity physics
        Rigidbody rb = fragmentObj.GetComponent<Rigidbody>();
        if (rb) DestroyImmediate(rb);

        BoxCollider col = fragmentObj.GetComponent<BoxCollider>();
        if (col) col.enabled = false;

        // Add your custom fragment script (if you have one)
        Fragment frag = fragmentObj.AddComponent<Fragment>();

        // Mass handling
        if (useDensity)
        {
            float volume = cubeSize * cubeSize * cubeSize; // m³
            frag.mass = density * volume;
        }
        else
        {
            frag.mass = fragmentMass;
        }

        frag.size = Vector3.one * cubeSize;

        // Apply material
        Renderer renderer = fragmentObj.GetComponent<Renderer>();
        if (fragmentMaterial != null)
        {
            renderer.material = new Material(fragmentMaterial);
        }

        // Optional: randomize color
        if (randomizeColors)
        {
            renderer.material.color = new Color(
                Random.Range(0.3f, 1f),
                Random.Range(0.3f, 1f),
                Random.Range(0.3f, 1f),
                1f
            );
        }

        return fragmentObj;
    }

    // ────────────────────────────────────────────────────────────────
    // Gizmos for visualization
    // ────────────────────────────────────────────────────────────────
    void OnDrawGizmos()
    {
        if (fragments == null) return;

        // Draw cubes and constraints between neighbors
        Gizmos.color = Color.yellow;
        foreach (GameObject frag in fragments)
        {
            if (frag == null) continue;
            Gizmos.DrawWireCube(frag.transform.position, Vector3.one * cubeSize * 0.98f);
        }

        // Draw neighbor links to visualize constraints
        Gizmos.color = new Color(0, 1, 0, 0.3f);
        for (int x = 0; x < gridSize; x++)
        {
            for (int y = 0; y < gridSize; y++)
            {
                for (int z = 0; z < gridSize; z++)
                {
                    GameObject f = fragments[x, y, z];
                    if (!f) continue;

                    Vector3 p = f.transform.position;

                    // Connect to +X, +Y, +Z neighbors (so no duplicates)
                    if (x + 1 < gridSize) Gizmos.DrawLine(p, fragments[x + 1, y, z].transform.position);
                    if (y + 1 < gridSize) Gizmos.DrawLine(p, fragments[x, y + 1, z].transform.position);
                    if (z + 1 < gridSize) Gizmos.DrawLine(p, fragments[x, y, z + 1].transform.position);
                }
            }
        }
    }
}
