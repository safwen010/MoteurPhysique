using UnityEngine;
using System.Collections.Generic;

public class PlatformFractureSimulator : MonoBehaviour, IFragmentSimulator
{
    [Header("Scene Objects")]
    public Transform ground;
    public Transform bigBall;

    [Header("Fracture Parameters")]
    [Range(0f, 2f)]
    public float alpha = 0.0f; // Energy multiplier for fragments
    public float baseFragmentSpeed = 8.0f;
    public float platformFallSpeed = 3.0f;

    [Header("Visual Settings")]
    public Color groundColor = new Color(0.3f, 0.2f, 0.1f); // Dark brown
    public Color ballColor = new Color(0.8f, 0.3f, 0.8f); // Purple
    public Color[] platformColors = new Color[]
    {
        new Color(1f, 0.5f, 0.2f), // Orange
        new Color(0.2f, 0.8f, 0.2f), // Green
        new Color(0.2f, 0.5f, 1f), // Blue
        new Color(1f, 0.8f, 0.2f), // Yellow
        new Color(0.8f, 0.2f, 0.5f), // Pink
        new Color(0.5f, 0.2f, 0.8f), // Purple
        new Color(0.2f, 0.8f, 0.8f), // Cyan
        new Color(0.8f, 0.8f, 0.2f), // Lime
        new Color(0.8f, 0.5f, 0.2f)  // Gold
    };

    [Header("Platform Settings")]
    public float platformSpacing = 1.2f;
    public float cubeSize = 0.8f;

    private List<FragmentPhysics> platformCubes = new List<FragmentPhysics>();
    private List<FragmentPhysics> platformFragments = new List<FragmentPhysics>(); // Additional fragments for explosion
    private bool simulationStarted = false;
    private float simulationTime = 0f;
    private FragmentPhysics ballPhysics;
    private Vector3 initialPlatformPosition;
    private bool collisionOccurred = false;

    void Start()
    {
        SetupScene();
        CreatePlatform();
        CreatePlatformFragments();
    }

    void SetupScene()
    {
        // Set up ground
        ground.GetComponent<Renderer>().material.color = groundColor;

        // Set up big ball - positioned higher at 4f
        bigBall.GetComponent<Renderer>().material.color = ballColor;
        bigBall.position = new Vector3(0f, 4f, 0f); // Increased height to 4f

        // Remove ALL Unity physics components
        RemoveAllUnityPhysics();

        // Store initial platform position (above the ball)
        initialPlatformPosition = new Vector3(0f, 8f, 0f); // Platform starts above the ball at 8f
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

    void CreatePlatform()
    {
        // Create 3x3 platform of cubes
        int index = 0;
        for (int x = -1; x <= 1; x++)
        {
            for (int z = -1; z <= 1; z++)
            {
                CreatePlatformCube(x, z, index);
                index++;
            }
        }
    }

    void CreatePlatformCube(int gridX, int gridZ, int colorIndex)
    {
        GameObject cubeObj = GameObject.CreatePrimitive(PrimitiveType.Cube);
        cubeObj.name = $"PlatformCube_{gridX}_{gridZ}";

        // Remove Unity physics components
        DestroyImmediate(cubeObj.GetComponent<Collider>());
        DestroyImmediate(cubeObj.GetComponent<BoxCollider>());

        // Position in grid
        Vector3 position = initialPlatformPosition +
                          new Vector3(gridX * platformSpacing, 0, gridZ * platformSpacing);
        cubeObj.transform.position = position;
        cubeObj.transform.localScale = Vector3.one * cubeSize;

        // Set color
        Renderer renderer = cubeObj.GetComponent<Renderer>();
        renderer.material = new Material(renderer.material);
        Color cubeColor = platformColors[colorIndex % platformColors.Length];
        renderer.material.color = cubeColor;

        // Add fragment physics
        FragmentPhysics fragment = cubeObj.AddComponent<FragmentPhysics>();
        fragment.groundY = ground.position.y + ground.localScale.y * 0.5f;
        fragment.SetSimulator(this);

        // Start with downward velocity to make platform fall
        fragment.velocity = Vector3.down * platformFallSpeed;

        platformCubes.Add(fragment);
    }

    void CreatePlatformFragments()
    {
        // Create additional fragments for explosive effect (smaller cubes)
        int fragmentCount = 16;
        float fragmentSize = 0.3f;

        for (int i = 0; i < fragmentCount; i++)
        {
            GameObject fragmentObj = GameObject.CreatePrimitive(PrimitiveType.Cube);
            fragmentObj.name = $"PlatformFragment_{i}";

            // Remove Unity physics components
            DestroyImmediate(fragmentObj.GetComponent<Collider>());
            DestroyImmediate(fragmentObj.GetComponent<BoxCollider>());

            fragmentObj.transform.position = initialPlatformPosition + Random.insideUnitSphere * 1f;
            fragmentObj.transform.localScale = Vector3.one * fragmentSize;

            // Create new material instance with random platform color
            Renderer renderer = fragmentObj.GetComponent<Renderer>();
            renderer.material = new Material(renderer.material);
            Color fragmentColor = platformColors[Random.Range(0, platformColors.Length)];
            renderer.material.color = fragmentColor;

            FragmentPhysics fragment = fragmentObj.AddComponent<FragmentPhysics>();
            fragment.groundY = ground.position.y + ground.localScale.y * 0.5f;
            fragment.SetSimulator(this);

            platformFragments.Add(fragment);
            fragmentObj.SetActive(false); // Start inactive
        }

        // Add physics to the main ball and make it static
        ballPhysics = bigBall.gameObject.AddComponent<FragmentPhysics>();
        ballPhysics.groundY = ground.position.y + ground.localScale.y * 0.5f;
        ballPhysics.SetSimulator(this);
        ballPhysics.gravity = 0f; // Ball doesn't fall
        ballPhysics.velocity = Vector3.zero; // Ball doesn't move
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
        collisionOccurred = false;
        Debug.Log($"Platform Fracture Simulation Started! Alpha = {alpha}");
    }

    void UpdateSimulation(float deltaTime)
    {
        // Update platform cubes
        foreach (FragmentPhysics cube in platformCubes)
        {
            if (cube.gameObject.activeInHierarchy)
            {
                cube.UpdatePhysics(deltaTime);

                // Check for collision with big ball
                if (!cube.collided && CheckCubeBallCollision(cube))
                {
                    OnPlatformBallCollision(cube);
                    cube.collided = true;
                }
            }
        }

        // Update ball physics
        if (ballPhysics != null && bigBall.gameObject.activeInHierarchy)
        {
            ballPhysics.UpdatePhysics(deltaTime);
        }

        // Update platform fragments
        foreach (FragmentPhysics fragment in platformFragments)
        {
            if (fragment.gameObject.activeInHierarchy)
            {
                fragment.UpdatePhysics(deltaTime);
            }
        }
    }

    bool CheckCubeBallCollision(FragmentPhysics cube)
    {
        if (!bigBall.gameObject.activeInHierarchy) return false;

        Vector3 cubePos = cube.transform.position;
        Vector3 ballPos = bigBall.position;
        float cubeHalfSize = cubeSize * 0.5f;
        float ballRadius = bigBall.localScale.x * 0.5f;

        // Simple sphere-AABB collision
        Vector3 closestPoint = new Vector3(
            Mathf.Clamp(cubePos.x, ballPos.x - cubeHalfSize, ballPos.x + cubeHalfSize),
            Mathf.Clamp(cubePos.y, ballPos.y - cubeHalfSize, ballPos.y + cubeHalfSize),
            Mathf.Clamp(cubePos.z, ballPos.z - cubeHalfSize, ballPos.z + cubeHalfSize)
        );

        float distance = Vector3.Distance(closestPoint, ballPos);
        return distance <= ballRadius;
    }

    void OnPlatformBallCollision(FragmentPhysics collidedCube)
    {
        if (collisionOccurred) return; // Only process first collision
        collisionOccurred = true;

        Vector3 collisionPoint = collidedCube.transform.position;

        // Apply alpha-based behavior
        if (alpha > 0.1f)
        {
            // EXPLOSIVE BEHAVIOR - Platform cubes explode
            ExplodePlatform(collisionPoint);
        }
        else
        {
            // Realistic behavior - just bounce naturally
            ApplyRealisticCollisionToAllCubes(collidedCube);
        }

        // Visual feedback
        StartCoroutine(FlashObject(collidedCube.gameObject));
    }

    void ExplodePlatform(Vector3 explosionCenter)
    {
        Debug.Log($"Platform exploding with alpha = {alpha}!");

        // Apply explosive forces to all platform cubes
        foreach (FragmentPhysics cube in platformCubes)
        {
            if (cube.gameObject.activeInHierarchy)
            {
                Vector3 direction = (cube.transform.position - explosionCenter).normalized;
                if (direction.magnitude < 0.1f) direction = Random.onUnitSphere;

                // Strong explosive force based on alpha - REPLACE velocity for dramatic effect
                float force = baseFragmentSpeed * alpha * Random.Range(0.8f, 1.5f);
                cube.velocity = direction * force; // REPLACE instead of add

                // Add explosive rotation
                cube.angularVelocity = Random.onUnitSphere * alpha * 10.0f;
            }
        }

        // Activate additional platform fragments for more dramatic explosion
        ActivatePlatformFragments(explosionCenter);
    }

    void ActivatePlatformFragments(Vector3 explosionCenter)
    {
        foreach (FragmentPhysics fragment in platformFragments)
        {
            if (!fragment.gameObject.activeInHierarchy)
            {
                fragment.gameObject.SetActive(true);
                fragment.InitializeVertices();

                // Position fragments around explosion center
                Vector3 randomOffset = Random.onUnitSphere * 1.5f;
                fragment.transform.position = explosionCenter + randomOffset;

                // Calculate explosion direction
                Vector3 direction = (fragment.transform.position - explosionCenter).normalized;
                if (direction.magnitude < 0.1f)
                    direction = Random.onUnitSphere;

                // Apply velocity based on alpha
                float speed = baseFragmentSpeed * alpha * Random.Range(0.5f, 1.5f);
                fragment.velocity = direction * speed;

                // Add rotation
                fragment.angularVelocity = Random.onUnitSphere * alpha * 12.0f;

                fragment.isGrounded = false;
                fragment.collided = false;
            }
        }
    }

    void ApplyRealisticCollisionToAllCubes(FragmentPhysics firstCollidedCube)
    {
        Debug.Log("Realistic collision - platform bouncing naturally");

        // Apply realistic bounce to all cubes
        foreach (FragmentPhysics cube in platformCubes)
        {
            if (cube.gameObject.activeInHierarchy)
            {
                // Simple bounce with restitution
                if (cube.velocity.y < 0)
                {
                    cube.velocity.y = -cube.velocity.y * 0.4f;
                }

                // Small horizontal scatter
                cube.velocity.x += Random.Range(-1f, 1f);
                cube.velocity.z += Random.Range(-1f, 1f);

                // Small rotation
                cube.angularVelocity += Random.onUnitSphere * 1.0f;
            }
        }
    }

    System.Collections.IEnumerator FlashObject(GameObject obj)
    {
        Renderer renderer = obj.GetComponent<Renderer>();
        if (renderer != null)
        {
            Color originalColor = renderer.material.color;
            renderer.material.color = Color.white;
            yield return new WaitForSeconds(0.1f);
            renderer.material.color = originalColor;
        }
    }

    public float GetAlpha()
    {
        return alpha;
    }

    public List<FragmentPhysics> GetAllFragments()
    {
        List<FragmentPhysics> allFragments = new List<FragmentPhysics>();
        allFragments.AddRange(platformCubes);
        allFragments.AddRange(platformFragments);
        if (ballPhysics != null && bigBall.gameObject.activeInHierarchy)
        {
            allFragments.Add(ballPhysics);
        }
        return allFragments;
    }

    void OnGUI()
    {
        GUILayout.BeginArea(new Rect(10, 10, 350, 250));
        GUILayout.Label($"Platform Fracture Simulation - Alpha: {alpha:F1}");
        GUILayout.Label("Press SPACE to start simulation");
        GUILayout.Label($"Simulation Time: {simulationTime:F2}s");

        if (simulationStarted)
        {
            GUILayout.Label($"Active Cubes: {GetActivePlatformCubes()}/9");
            GUILayout.Label($"Active Fragments: {GetActivePlatformFragments()}/{platformFragments.Count}");
            GUILayout.Label($"Behavior: {(alpha < 0.1f ? "Realistic" : "Explosive")}");
            GUILayout.Label($"Explosion Strength: {alpha * 100f:F0}%");

            if (collisionOccurred)
            {
                GUILayout.Label("STATUS: COLLISION OCCURRED!");
            }
        }

        GUILayout.Label("Controls:");
        GUILayout.Label("- Alpha = 0: Realistic bounce");
        GUILayout.Label("- Alpha > 0: Platform explodes");
        GUILayout.Label("- Higher Alpha = Bigger explosion");

        GUILayout.EndArea();
    }

    int GetActivePlatformCubes()
    {
        int count = 0;
        foreach (FragmentPhysics cube in platformCubes)
        {
            if (cube.gameObject.activeInHierarchy) count++;
        }
        return count;
    }

    int GetActivePlatformFragments()
    {
        int count = 0;
        foreach (FragmentPhysics fragment in platformFragments)
        {
            if (fragment.gameObject.activeInHierarchy) count++;
        }
        return count;
    }
}