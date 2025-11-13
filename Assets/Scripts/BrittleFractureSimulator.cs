using UnityEngine;
using System.Collections.Generic;

public class BrittleFractureSimulator : MonoBehaviour, IFragmentSimulator
{
    [Header("Scene Objects")]
    public Transform ground;
    public Transform redBeam;
    public Transform blueCube;
    public Transform yellowProjectile;

    [Header("Fracture Parameters")]
    [Range(0f, 2f)]
    public float alpha = 1.0f;
    public float fractureThreshold = 2.0f;
    public float baseFragmentSpeed = 5.0f;

    [Header("Visual Settings")]
    public Color groundColor = new Color(1f, 0.8f, 0.2f);
    public Color beamColor = Color.red;
    public Color cubeColor = Color.blue;
    public Color projectileColor = Color.yellow;
    public Color fragmentColor = Color.green;

    private List<FragmentPhysics> fragments = new List<FragmentPhysics>();
    private bool simulationStarted = false;
    private float simulationTime = 0f;
    private FragmentPhysics projectilePhysics;
    private Vector3[] blueCubeVertices;

    void Start()
    {
        SetupScene();
        CreateFragments();
        PrecalculateCubeVertices();
    }

    void SetupScene()
    {
        ground.GetComponent<Renderer>().material.color = groundColor;
        redBeam.GetComponent<Renderer>().material.color = beamColor;
        blueCube.GetComponent<Renderer>().material.color = cubeColor;
        yellowProjectile.GetComponent<Renderer>().material.color = projectileColor;
        redBeam.position = new Vector3(2f, 0.5f, 0f);
        blueCube.position = new Vector3(0f, 0.5f, 0f);
        yellowProjectile.position = new Vector3(-3f, 0.5f, 0f);
        RemoveAllUnityPhysics();
    }

    void RemoveAllUnityPhysics()
    {
        Collider[] colliders = FindObjectsOfType<Collider>();
        foreach (Collider col in colliders)
        {
            DestroyImmediate(col);
        }
        Rigidbody[] rigidbodies = FindObjectsOfType<Rigidbody>();
        foreach (Rigidbody rb in rigidbodies)
        {
            DestroyImmediate(rb);
        }
    }

    void PrecalculateCubeVertices()
    {
        Vector3 halfSize = blueCube.localScale * 0.5f;
        blueCubeVertices = new Vector3[]
        {
            new Vector3(-halfSize.x, -halfSize.y, -halfSize.z),
            new Vector3(-halfSize.x, -halfSize.y,  halfSize.z),
            new Vector3(-halfSize.x,  halfSize.y, -halfSize.z),
            new Vector3(-halfSize.x,  halfSize.y,  halfSize.z),
            new Vector3( halfSize.x, -halfSize.y, -halfSize.z),
            new Vector3( halfSize.x, -halfSize.y,  halfSize.z),
            new Vector3( halfSize.x,  halfSize.y, -halfSize.z),
            new Vector3( halfSize.x,  halfSize.y,  halfSize.z)
        };
    }

    void CreateFragments()
    {
        CreateCubeFragments(blueCube.position, cubeColor);
        CreateSecondaryFragments(blueCube.position + Vector3.up, fragmentColor);
    }

    void CreateCubeFragments(Vector3 position, Color color)
    {
        int fragmentCount = 8;
        float size = 0.3f;

        for (int i = 0; i < fragmentCount; i++)
        {
            GameObject fragmentObj = GameObject.CreatePrimitive(PrimitiveType.Cube);
            fragmentObj.name = $"CubeFragment_{i}";

            DestroyImmediate(fragmentObj.GetComponent<Collider>());
            DestroyImmediate(fragmentObj.GetComponent<BoxCollider>());

            fragmentObj.transform.position = position + Random.insideUnitSphere * 0.3f;
            fragmentObj.transform.localScale = Vector3.one * size;

            Renderer renderer = fragmentObj.GetComponent<Renderer>();
            renderer.material = new Material(renderer.material);
            renderer.material.color = color;

            FragmentPhysics fragment = fragmentObj.AddComponent<FragmentPhysics>();
            fragment.groundY = ground.position.y + ground.localScale.y * 0.5f;
            fragment.SetSimulator(this);

            fragments.Add(fragment);
            fragmentObj.SetActive(false);
        }
    }

    void CreateSecondaryFragments(Vector3 position, Color color)
    {
        int fragmentCount = 1;
        float size = 0.2f;

        for (int i = 0; i < fragmentCount; i++)
        {
            GameObject fragmentObj = GameObject.CreatePrimitive(PrimitiveType.Sphere);
            fragmentObj.name = $"SecondaryFragment_{i}";

            DestroyImmediate(fragmentObj.GetComponent<Collider>());
            DestroyImmediate(fragmentObj.GetComponent<SphereCollider>());

            fragmentObj.transform.position = position + Random.insideUnitSphere * 0.2f;
            fragmentObj.transform.localScale = Vector3.one * size;

            Renderer renderer = fragmentObj.GetComponent<Renderer>();
            renderer.material = new Material(renderer.material);
            renderer.material.color = color;

            FragmentPhysics fragment = fragmentObj.AddComponent<FragmentPhysics>();
            fragment.groundY = ground.position.y + ground.localScale.y * 0.5f;
            fragment.SetSimulator(this);

            fragments.Add(fragment);
            fragmentObj.SetActive(false);
        }
    }

    void Update()
    {
        if (!simulationStarted && Input.GetKeyDown(KeyCode.Space))
        {
            StartSimulation();
        }

        if (simulationStarted)
        {
            simulationTime += Time.deltaTime;
            UpdateSimulation(Time.deltaTime);
        }
    }

    void StartSimulation()
    {
        simulationStarted = true;
        simulationTime = 0f;

        LaunchProjectile();
    }

    void LaunchProjectile()
    {
        Vector3 direction = (blueCube.position - yellowProjectile.position).normalized;
        float speed = 10.0f;

        projectilePhysics = yellowProjectile.gameObject.AddComponent<FragmentPhysics>();
        projectilePhysics.velocity = direction * speed;
        projectilePhysics.groundY = ground.position.y + ground.localScale.y * 0.5f;
        projectilePhysics.SetSimulator(this);
    }

    void UpdateSimulation(float deltaTime)
    {
        if (projectilePhysics != null && !projectilePhysics.collided)
        {
            projectilePhysics.UpdatePhysics(deltaTime);

            float projectileRadius = GetProjectileRadius();

            Matrix4x4 cubeRotMatrix = Matrix4x4.TRS(Vector3.zero, blueCube.rotation, Vector3.one);

            if (CheckMeshCollision(projectilePhysics.transform.position, projectileRadius,
                                   blueCube.position, blueCube.localScale * 0.5f,
                                   cubeRotMatrix)) 
            {
                OnImpact();
                projectilePhysics.collided = true;
            }
        }
        foreach (FragmentPhysics fragment in fragments)
        {
            if (fragment.gameObject.activeInHierarchy)
                fragment.UpdatePhysics(deltaTime);
        }
        UpdateRedBeam();
    }

    float GetProjectileRadius()
    {
        return yellowProjectile.localScale.x * 0.5f;
    }

    bool CheckMeshCollision(
    Vector3 spherePos, float sphereRadius,
    Vector3 cubePos, Vector3 cubeHalfExtents,
    Matrix4x4 cubeRotMat)  
    {
        Matrix4x4 cubeInv = cubeRotMat.transpose;
        Vector3 localSpherePos = cubeInv.MultiplyPoint3x4(spherePos - cubePos);

        Vector3 closest = new Vector3(
            Mathf.Clamp(localSpherePos.x, -cubeHalfExtents.x, cubeHalfExtents.x),
            Mathf.Clamp(localSpherePos.y, -cubeHalfExtents.y, cubeHalfExtents.y),
            Mathf.Clamp(localSpherePos.z, -cubeHalfExtents.z, cubeHalfExtents.z)
        );

        return Vector3.Distance(localSpherePos, closest) <= sphereRadius;
    }

    void OnImpact()
    {
        blueCube.gameObject.SetActive(false);
        yellowProjectile.gameObject.SetActive(false);

        Vector3 impactPoint = blueCube.position;
        ActivateFragments(impactPoint);

        Debug.Log($"Impact! Alpha = {alpha}");
    }

    void ActivateFragments(Vector3 impactPoint)
    {
        foreach (FragmentPhysics fragment in fragments)
        {
            if (!fragment.gameObject.activeInHierarchy)
            {
                fragment.gameObject.SetActive(true);
                fragment.InitializeVertices();

                Vector3 randomOffset = Random.onUnitSphere * 0.4f;
                fragment.transform.position = impactPoint + randomOffset;

                Vector3 direction = (fragment.transform.position - impactPoint).normalized;
                if (direction.magnitude < 0.1f)
                    direction = Random.onUnitSphere;

                float minSpeed = 0.3f; 
                float alphaSpeed = baseFragmentSpeed * alpha * Random.Range(0.8f, 1.2f);
                float speed = Mathf.Max(alphaSpeed, minSpeed);

                fragment.velocity = direction * speed;

                float minRotation = 0.5f;
                float alphaRotation = alpha * 5.0f;
                float rotationStrength = Mathf.Max(alphaRotation, minRotation);
                fragment.angularVelocity = Random.onUnitSphere * rotationStrength;

                fragment.isGrounded = false;
                fragment.collided = false;
            }
        }
    }
    void UpdateRedBeam()
    {
        Vector3 totalInfluence = Vector3.zero;
        int influenceCount = 0;

        foreach (FragmentPhysics fragment in fragments)
        {
            if (fragment.gameObject.activeInHierarchy && !fragment.isGrounded)
            {
                float distance = Vector3.Distance(fragment.transform.position, redBeam.position);
                if (distance < 1.5f)
                {
                    Vector3 direction = (fragment.transform.position - redBeam.position).normalized;
                    float influence = (1.5f - distance) / 1.5f;
                    totalInfluence += direction * influence * alpha;
                    influenceCount++;
                }
            }
        }

        if (influenceCount > 0 && totalInfluence.magnitude > 0.1f)
        {
            FragmentPhysics beamPhysics = redBeam.GetComponent<FragmentPhysics>();
            if (beamPhysics == null)
            {
                beamPhysics = redBeam.gameObject.AddComponent<FragmentPhysics>();
                beamPhysics.groundY = ground.position.y + ground.localScale.y * 0.5f;
                beamPhysics.SetSimulator(this);
            }

            if (!beamPhysics.isGrounded)
            {
                beamPhysics.velocity += totalInfluence * 2.0f * Time.deltaTime;
            }
        }
    }

    public float GetAlpha()
    {
        return alpha;
    }

    void OnGUI()
    {
        GUILayout.BeginArea(new Rect(10, 10, 300, 200));
        GUILayout.Label($"Brittle Fracture Simulation - Alpha: {alpha:F1}");
        GUILayout.Label("Press SPACE to start simulation");
        GUILayout.Label($"Simulation Time: {simulationTime:F2}s");

        if (simulationStarted)
        {
            GUILayout.Label("Active Fragments: " + GetActiveFragmentCount());
            GUILayout.Label("Fragment Speed Multiplier: " + alpha);
        }

        GUILayout.EndArea();
    }

    int GetActiveFragmentCount()
    {
        int count = 0;
        foreach (FragmentPhysics fragment in fragments)
        {
            if (fragment.gameObject.activeInHierarchy) count++;
        }
        return count;
    }
    public List<FragmentPhysics> GetAllFragments()
    {
        return fragments;
    }
}

public enum MeshType
{
    Cube,
    Sphere,
    Cylinder
}