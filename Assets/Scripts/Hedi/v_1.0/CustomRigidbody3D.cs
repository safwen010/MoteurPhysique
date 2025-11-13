using System;
using UnityEngine;

[DisallowMultipleComponent]
public class CustomRigidbody3D : MonoBehaviour
{
    [Header("Physics Properties")]
    public float mass = 1f;
    public bool useGravity = true;
    public Vector3 velocity;
    public Vector3 acceleration;
    // Angular velocity (degrees per second) for simple rotation simulation
    public Vector3 angularVelocity;

    // Freeze constraints (similar to Unity's RigidbodyConstraints)
    [Header("Constraints")]
    public bool freezePositionX = false;
    public bool freezePositionY = false;
    public bool freezePositionZ = false;
    public bool freezeRotationX = false;
    public bool freezeRotationY = false;
    public bool freezeRotationZ = false;

    private Vector3 forces;
    // Locked pose values used when a constraint is active
    private Vector3 lockedPosition;
    private Vector3 lockedEuler;

    public void AddForce(Vector3 force)
    {
        forces += force;
    }

    public void PhysicsUpdate(float deltaTime)
    {
        if (useGravity)
            AddForce(Physics.gravity * mass);

        // Integrate
        acceleration = forces / mass;
        velocity += acceleration * deltaTime;
        transform.position += velocity * deltaTime;

        // Integrate simple angular motion (degrees)
        if (angularVelocity != Vector3.zero)
        {
            transform.Rotate(angularVelocity * deltaTime, Space.Self);
        }

        // Reset forces
        forces = Vector3.zero;

        // Enforce positional constraints: keep locked axes at the stored lockedPosition
        Vector3 pos = transform.position;
        if (freezePositionX) { pos.x = lockedPosition.x; velocity.x = 0f; }
        if (freezePositionY) { pos.y = lockedPosition.y; velocity.y = 0f; }
        if (freezePositionZ) { pos.z = lockedPosition.z; velocity.z = 0f; }
        transform.position = pos;

        // Enforce rotational constraints: keep locked Euler components and zero angular velocity on frozen axes
        Vector3 euler = transform.eulerAngles;
        if (freezeRotationX) { euler.x = lockedEuler.x; angularVelocity.x = 0f; }
        if (freezeRotationY) { euler.y = lockedEuler.y; angularVelocity.y = 0f; }
        if (freezeRotationZ) { euler.z = lockedEuler.z; angularVelocity.z = 0f; }
        transform.eulerAngles = euler;
    }

    void Start()
    {
        // Initialize locked pose to the current transform so freezes hold the current pose
        lockedPosition = transform.position;
        lockedEuler = transform.eulerAngles;
    }
}
