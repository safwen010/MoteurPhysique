using System.Collections.Generic;
using UnityEngine;

public class CustomPhysicsManager : MonoBehaviour
{
    private List<CustomRigidbody3D> bodies = new List<CustomRigidbody3D>();
    private List<CustomCollider3D> colliders = new List<CustomCollider3D>();

    private static CustomPhysicsManager instance;

    public static CustomPhysicsManager Instance
    {
        get
        {
            if (instance == null)
            {
                var go = new GameObject("CustomPhysicsManager");
                instance = go.AddComponent<CustomPhysicsManager>();
            }
            return instance;
        }
    }

    void Awake()
    {
        instance = this;
    }

    public void Register(CustomRigidbody3D rb, CustomCollider3D col)
    {
        if (!bodies.Contains(rb)) bodies.Add(rb);
        if (!colliders.Contains(col)) colliders.Add(col);
    }

    void Update()
    {
        Simulate(Time.deltaTime);
    }

    void Simulate(float deltaTime)
    {
        for (int i = 0; i < bodies.Count; i++)
        {
            bodies[i].PhysicsUpdate(deltaTime);
        }

        // basic AABB collision detection (works for all CustomCollider3D via GetAABB)
        for (int i = 0; i < colliders.Count; i++)
        {
            for (int j = i + 1; j < colliders.Count; j++)
            {
                var colA = colliders[i];
                var colB = colliders[j];

                if (colA.Intersects(colB))
                {
                    var rbA = colA.GetComponent<CustomRigidbody3D>();
                    var rbB = colB.GetComponent<CustomRigidbody3D>();

                    // Use AABBs provided by colliders
                    colA.GetAABB(out Vector3 aCenter, out Vector3 aSize);
                    colB.GetAABB(out Vector3 bCenter, out Vector3 bSize);
                    Vector3 halfA = aSize * 0.5f;
                    Vector3 halfB = bSize * 0.5f;
                    Vector3 delta = bCenter - aCenter;

                    float overlapX = (halfA.x + halfB.x) - Mathf.Abs(delta.x);
                    float overlapY = (halfA.y + halfB.y) - Mathf.Abs(delta.y);
                    float overlapZ = (halfA.z + halfB.z) - Mathf.Abs(delta.z);

                    // Choose the minimum penetration axis to resolve
                    Vector3 normal = Vector3.zero;
                    float minOverlap = overlapX;
                    normal = new Vector3(delta.x < 0 ? -1f : 1f, 0f, 0f);
                    if (overlapY < minOverlap)
                    {
                        minOverlap = overlapY;
                        normal = new Vector3(0f, delta.y < 0 ? -1f : 1f, 0f);
                    }
                    if (overlapZ < minOverlap)
                    {
                        minOverlap = overlapZ;
                        normal = new Vector3(0f, 0f, delta.z < 0 ? -1f : 1f);
                    }

                    // Penetration vector (from A out of B)
                    Vector3 penetration = normal * minOverlap;

                    // Inverse mass (static object => invMass 0)
                    float invA = (rbA != null) ? 1f / Mathf.Max(0.0001f, rbA.mass) : 0f;
                    float invB = (rbB != null) ? 1f / Mathf.Max(0.0001f, rbB.mass) : 0f;
                    float invSum = invA + invB;

                    // Positional correction: move objects out of penetration
                    if (rbA != null)
                    {
                        Vector3 moveA = -penetration * (invA / Mathf.Max(0.0001f, invSum));
                        rbA.transform.position += moveA;
                    }
                    if (rbB != null)
                    {
                        Vector3 moveB = penetration * (invB / Mathf.Max(0.0001f, invSum));
                        rbB.transform.position += moveB;
                    }

                    // Velocity correction along collision normal (simple impulse/damping)
                    float restitution = 0.2f; // bounce
                    if (rbA != null)
                    {
                        float velAlong = Vector3.Dot(rbA.velocity, normal);
                        if (velAlong < 0f)
                        {
                            rbA.velocity -= normal * (velAlong * (1f + restitution));
                        }
                    }
                    if (rbB != null)
                    {
                        float velAlong = Vector3.Dot(rbB.velocity, normal);
                        if (velAlong > 0f)
                        {
                            rbB.velocity -= normal * (velAlong * (1f + restitution));
                        }
                    }

                    // Apply a small damping to avoid jitter and re-penetration
                    if (rbA != null) rbA.velocity *= 0.9f;
                    if (rbB != null) rbB.velocity *= 0.9f;
                }
            }
        }
    }
}
