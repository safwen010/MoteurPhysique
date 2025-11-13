using UnityEngine;
using System.Collections.Generic;

public interface IFragmentSimulator
{
    List<FragmentPhysics> GetAllFragments();
    float GetAlpha();
}

public class FragmentPhysics : MonoBehaviour
{
    [Header("Physics Properties")]
    public Vector3 velocity = Vector3.zero;
    public Vector3 angularVelocity = Vector3.zero;
    public float gravity = -9.81f;
    public float groundFriction = 0.8f;
    public float airResistance = 0.995f;
    public float groundRestitution = 0.3f;
    public float fragmentRestitution = 0.6f;

    [Header("Stabilization")]
    public bool enableStabilization = true;
    public float stabilizationForce = 2.0f;
    public float stabilizationTorque = 1.0f;

    [Header("Runtime Info")]
    public bool isGrounded = false;
    public bool collided = false;
    public float groundY = 0f;

    private IFragmentSimulator simulator;
    private Vector3 lastPosition;
    private Vector3[] meshVertices;
    private bool verticesInitialized = false;
    private Mesh mesh;
    private float timeGrounded = 0f;
    private Bounds meshBounds;

    public void SetSimulator(IFragmentSimulator sim)
    {
        simulator = sim;
    }
    public void SetSimulator(BrittleFractureSimulator sim)
    {
        simulator = sim;
    }

    public void SetSimulator(PlatformFractureSimulator sim)
    {
        simulator = sim;
    }

    void Awake()
    {
        MeshFilter meshFilter = GetComponent<MeshFilter>();
        if (meshFilter != null && meshFilter.mesh != null)
        {
            mesh = meshFilter.mesh;
        }

        PrecalculateMeshVertices();
        verticesInitialized = true;
        meshBounds = GetMeshBounds();
    }

    void Start()
    {
        lastPosition = transform.position;
        if (!verticesInitialized)
        {
            PrecalculateMeshVertices();
        }
    }

    void PrecalculateMeshVertices()
    {
        if (mesh != null)
        {
            meshVertices = mesh.vertices;
        }
        else
        {
            CalculatePrimitiveVertices();
        }
        verticesInitialized = true;
        meshBounds = GetMeshBounds();
    }

    void CalculatePrimitiveVertices()
    {
        PrimitiveType primitiveType = GetPrimitiveType();
        Vector3 scale = transform.localScale;

        switch (primitiveType)
        {
            case PrimitiveType.Cube:
                CalculateCubeVertices(scale);
                break;
            case PrimitiveType.Sphere:
                CalculateSphereVertices(scale);
                break;
            case PrimitiveType.Cylinder:
                CalculateCylinderVertices(scale);
                break;
            default:
                CalculateCubeVertices(scale);
                break;
        }
    }

    PrimitiveType GetPrimitiveType()
    {
        MeshFilter meshFilter = GetComponent<MeshFilter>();
        if (meshFilter != null && meshFilter.mesh != null)
        {
            string meshName = meshFilter.mesh.name.ToLower();
            if (meshName.Contains("sphere")) return PrimitiveType.Sphere;
            if (meshName.Contains("cylinder") || meshName.Contains("capsule")) return PrimitiveType.Cylinder;
        }
        return PrimitiveType.Cube;
    }

    void CalculateCubeVertices(Vector3 scale)
    {
        Vector3 halfSize = scale * 0.5f;
        meshVertices = new Vector3[]
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

    void CalculateSphereVertices(Vector3 scale)
    {
        float radius = Mathf.Max(scale.x, scale.y, scale.z) * 0.5f;
        List<Vector3> vertices = new List<Vector3>();

        int stacks = 3;
        int slices = 6;

        for (int i = 0; i <= stacks; ++i)
        {
            float phi = Mathf.PI * i / stacks;
            for (int j = 0; j <= slices; ++j)
            {
                float theta = 2 * Mathf.PI * j / slices;
                Vector3 point = new Vector3(
                    Mathf.Sin(phi) * Mathf.Cos(theta),
                    Mathf.Cos(phi),
                    Mathf.Sin(phi) * Mathf.Sin(theta)
                ) * radius;
                vertices.Add(point);
            }
        }

        meshVertices = vertices.ToArray();
    }

    void CalculateCylinderVertices(Vector3 scale)
    {
        float radius = Mathf.Max(scale.x, scale.z) * 0.5f;
        float halfHeight = scale.y * 0.5f;
        List<Vector3> vertices = new List<Vector3>();

        int segments = 12;
        vertices.Add(new Vector3(0, halfHeight, 0));
        for (int i = 0; i < segments; i++)
        {
            float angle = i * Mathf.PI * 2f / segments;
            vertices.Add(new Vector3(
                Mathf.Cos(angle) * radius,
                halfHeight,
                Mathf.Sin(angle) * radius
            ));
        }

        vertices.Add(new Vector3(0, -halfHeight, 0));
        for (int i = 0; i < segments; i++)
        {
            float angle = i * Mathf.PI * 2f / segments;
            vertices.Add(new Vector3(
                Mathf.Cos(angle) * radius,
                -halfHeight,
                Mathf.Sin(angle) * radius
            ));
        }

        for (int i = 0; i < segments; i++)
        {
            float angle = i * Mathf.PI * 2f / segments;
            vertices.Add(new Vector3(
                Mathf.Cos(angle) * radius,
                halfHeight,
                Mathf.Sin(angle) * radius
            ));
            vertices.Add(new Vector3(
                Mathf.Cos(angle) * radius,
                -halfHeight,
                Mathf.Sin(angle) * radius
            ));
        }

        meshVertices = vertices.ToArray();
    }

    public void UpdatePhysics(float deltaTime)
    {
        if (!verticesInitialized || meshVertices == null)
        {
            PrecalculateMeshVertices();
        }

        if (isGrounded)
        {
            timeGrounded += deltaTime;
        }
        else
        {
            timeGrounded = 0f;
        }

        if (isGrounded && velocity.magnitude < 0.1f && enableStabilization && timeGrounded > 0.5f)
        {
            ApplyStabilization(deltaTime);
        }

        if (!isGrounded)
        {
            velocity.y += gravity * deltaTime;
        }

        velocity *= airResistance;
        angularVelocity *= airResistance;

        transform.position += velocity * deltaTime;

        if (angularVelocity.magnitude > 0.01f)
        {
            
            Vector3 axis = angularVelocity.normalized;
            float angleRad = angularVelocity.magnitude * deltaTime;

            float c = Mathf.Cos(angleRad);
            float s = Mathf.Sin(angleRad);
            float omc = 1f - c;
            float x = axis.x, y = axis.y, z = axis.z;

            Matrix4x4 R = Matrix4x4.zero;
            R[0, 0] = c + x * x * omc; R[0, 1] = x * y * omc - z * s; R[0, 2] = x * z * omc + y * s;
            R[1, 0] = y * x * omc + z * s; R[1, 1] = c + y * y * omc; R[1, 2] = y * z * omc - x * s;
            R[2, 0] = z * x * omc - y * s; R[2, 1] = z * y * omc + x * s; R[2, 2] = c + z * z * omc;
            R[3, 3] = 1f;

            Matrix4x4 M = Matrix4x4.TRS(Vector3.zero, transform.rotation, Vector3.one);
            M = R * M;                                    // rotate
            transform.rotation = Quaternion.LookRotation(M.GetColumn(2), M.GetColumn(1)); // back to Unity rot
        }

        CheckGroundCollision();

        if (simulator != null)
        {
            CheckFragmentCollisions();
        }

        lastPosition = transform.position;
    }

    void CheckFragmentCollisions()
    {
        if (simulator == null) return;

        List<FragmentPhysics> allFragments = simulator.GetAllFragments();
        if (allFragments == null) return;

        foreach (FragmentPhysics otherFragment in allFragments)
        {
            if (otherFragment == this || !otherFragment.gameObject.activeInHierarchy)
                continue;

            if (CheckFragmentCollision(otherFragment))
            {
                ResolveFragmentCollision(otherFragment);
            }
        }
    }

    bool CheckFragmentCollision(FragmentPhysics other)
    {
        Vector3 toOther = other.transform.position - transform.position;
        float combinedRadius = GetBoundingSphereRadius() + other.GetBoundingSphereRadius();

        if (toOther.sqrMagnitude < combinedRadius * combinedRadius)
        {
            Bounds myWorldBounds = GetWorldBounds();
            Bounds otherWorldBounds = other.GetWorldBounds();

            return myWorldBounds.Intersects(otherWorldBounds);
        }

        return false;
    }

    void ResolveFragmentCollision(FragmentPhysics other)
    {
        Vector3 collisionNormal = (transform.position - other.transform.position).normalized;
        float penetration = CalculatePenetrationDepth(other);

        if (penetration > 0)
        {
            Vector3 separation = collisionNormal * penetration * 0.5f;
            transform.position += separation;
            other.transform.position -= separation;

            Vector3 relativeVelocity = velocity - other.velocity;
            float velocityAlongNormal = Vector3.Dot(relativeVelocity, collisionNormal);

            if (velocityAlongNormal < 0)
            {
                float impulseMagnitude = -(1 + fragmentRestitution) * velocityAlongNormal;
                impulseMagnitude /= 2;

                Vector3 impulse = impulseMagnitude * collisionNormal;
                velocity += impulse;
                other.velocity -= impulse;

                angularVelocity += Vector3.Cross(collisionNormal, impulse) * 0.1f;
                other.angularVelocity += Vector3.Cross(-collisionNormal, -impulse) * 0.1f;
            }
        }
    }

    float CalculatePenetrationDepth(FragmentPhysics other)
    {
        Bounds myBounds = GetWorldBounds();
        Bounds otherBounds = other.GetWorldBounds();

        if (!myBounds.Intersects(otherBounds))
            return 0f;

        Vector3 direction = other.transform.position - transform.position;
        Vector3 penetration = Vector3.zero;

        penetration.x = Mathf.Max(0, myBounds.extents.x + otherBounds.extents.x - Mathf.Abs(direction.x));
        penetration.y = Mathf.Max(0, myBounds.extents.y + otherBounds.extents.y - Mathf.Abs(direction.y));
        penetration.z = Mathf.Max(0, myBounds.extents.z + otherBounds.extents.z - Mathf.Abs(direction.z));

        return penetration.magnitude;
    }

    public float GetBoundingSphereRadius()
    {
        return meshBounds.extents.magnitude;
    }

    public Bounds GetWorldBounds()
    {
        Bounds worldBounds = new Bounds(transform.position, Vector3.zero);

        foreach (Vector3 vertex in meshVertices)
        {
            worldBounds.Encapsulate(transform.TransformPoint(vertex));
        }

        return worldBounds;
    }

    void ApplyStabilization(float deltaTime)
    {
        if (GetPrimitiveType() != PrimitiveType.Cube) return;

        Vector3 currentUp = transform.up;
        Vector3 targetUp = Vector3.up;

        float angle = Vector3.Angle(currentUp, targetUp);

        if (angle > 5f)
        {
            Vector3 rotationAxis = Vector3.Cross(currentUp, targetUp).normalized;

            float torqueStrength = stabilizationTorque * (angle / 90f);
            angularVelocity += rotationAxis * torqueStrength * deltaTime;

            angularVelocity *= 0.95f;
        }
    }

    void CheckGroundCollision()
    {
        if (!verticesInitialized || meshVertices == null || meshVertices.Length == 0)
        {
            PrecalculateMeshVertices();
            return;
        }

        CheckSimpleGroundCollision();
    }

    void CheckSimpleGroundCollision()
    {
        bool wasGrounded = isGrounded;
        float lowestPoint = float.MaxValue;
        int verticesTouching = 0;

        foreach (Vector3 vertex in meshVertices)
        {
            Vector3 worldVertex = transform.TransformPoint(vertex);
            if (worldVertex.y < lowestPoint)
            {
                lowestPoint = worldVertex.y;
            }

            if (Mathf.Abs(worldVertex.y - groundY) < 0.02f)
            {
                verticesTouching++;
            }
        }

        bool shouldBeGrounded = false;

        if (GetPrimitiveType() == PrimitiveType.Cube)
        {
            shouldBeGrounded = verticesTouching >= 2 && lowestPoint <= groundY + 0.05f && velocity.y <= 0;
        }
        else
        {
            shouldBeGrounded = lowestPoint <= groundY + 0.01f && velocity.y <= 0;
        }

        if (shouldBeGrounded)
        {
            if (!wasGrounded)
            {
                float penetration = groundY - lowestPoint;
                if (penetration > 0)
                {
                    transform.position += Vector3.up * penetration;
                }

                velocity.y = -velocity.y * groundRestitution;

                velocity.x *= groundFriction;
                velocity.z *= groundFriction;

                angularVelocity *= 0.7f;
            }

            if (Mathf.Abs(velocity.y) < 0.1f)
            {
                velocity.y = 0f;
            }

            isGrounded = true;
        }
        else
        {
            isGrounded = false;
        }

        if (lowestPoint < groundY - 0.01f)
        {
            float penetration = groundY - lowestPoint;
            transform.position += Vector3.up * penetration * 0.5f;

            if (velocity.y < 0)
            {
                velocity.y = -velocity.y * 0.2f;
            }
        }
    }

    Vector3[] GetWorldVertices()
    {
        if (!verticesInitialized || meshVertices == null)
        {
            PrecalculateMeshVertices();
        }

        Vector3[] worldVertices = new Vector3[meshVertices.Length];
        for (int i = 0; i < meshVertices.Length; i++)
        {
            worldVertices[i] = transform.TransformPoint(meshVertices[i]);
        }
        return worldVertices;
    }

    void OnDrawGizmosSelected()
    {
        if (!verticesInitialized || meshVertices == null)
        {
            PrecalculateMeshVertices();
        }

        Gizmos.color = isGrounded ? Color.green : Color.red;
        Vector3[] worldVertices = GetWorldVertices();
        foreach (Vector3 vertex in worldVertices)
        {
            Gizmos.DrawSphere(vertex, 0.02f);
        }

        Gizmos.color = Color.magenta;
        Bounds worldBounds = GetWorldBounds();
        Gizmos.DrawWireCube(worldBounds.center, worldBounds.size);

        Gizmos.color = Color.yellow;
        Gizmos.DrawLine(new Vector3(-5, groundY, 0), new Vector3(5, groundY, 0));

        Gizmos.color = Color.blue;
        Gizmos.DrawRay(transform.position, velocity * 0.1f);
    }

    Bounds GetMeshBounds()
    {
        if (!verticesInitialized || meshVertices == null || meshVertices.Length == 0)
            return new Bounds(Vector3.zero, Vector3.one);

        Vector3 min = meshVertices[0];
        Vector3 max = meshVertices[0];

        foreach (Vector3 vertex in meshVertices)
        {
            min = Vector3.Min(min, vertex);
            max = Vector3.Max(max, vertex);
        }

        return new Bounds((min + max) * 0.5f, max - min);
    }

    public void InitializeVertices()
    {
        PrecalculateMeshVertices();
    }
}