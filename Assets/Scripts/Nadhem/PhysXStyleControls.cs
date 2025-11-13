using UnityEngine;

/// <summary>
/// Simple, PhysX-style inspector to control the cube fracture simulation without exposing advanced parameters.
/// Attach this to the same GameObject as FractureManager. It will push values into FractureManager on change.
/// </summary>
[DisallowMultipleComponent]
public class PhysXStyleControls : MonoBehaviour
{
    [Header("PhysX-style")]
    [Tooltip("Gravity along Y (m/s^2). Negative for downward gravity.")]
    public float gravityY = -9.81f;

    [Tooltip("Bounciness (0..1). 0 = no bounce, 1 = perfect bounce.")]
    [Range(0f,1f)] public float restitution = 0.2f;

    [Tooltip("Friction (0..1). Higher = more resistance to sliding.")]
    [Range(0f,1f)] public float friction = 0.6f;

    [Tooltip("Normalized linear damping (0..0.1 typical). 0 = no damping. Internally mapped to multiplicative air damping.")]
    [Range(0f,0.2f)] public float linearDamping = 0.005f;

    [Tooltip("Break impulse threshold (N·s). Higher = harder to break.")]
    public float breakImpulse = 10f;

    [Tooltip("Minimum impact speed (m/s) required before constraints can break on contact.")]
    public float impactSpeedThreshold = 2f;

    [Tooltip("Spawn height above ground (m).")]
    public float dropHeight = 5f;

    [Tooltip("Initial linear velocity (m/s).")]
    public Vector3 initialVelocity = Vector3.zero;

    private FractureManager manager;

    void Awake()
    {
        TryGetComponent(out manager);
        Apply();
    }

    void OnValidate()
    {
        if (manager == null) TryGetComponent(out manager);
        Apply();
    }

    void Apply()
    {
        if (manager == null) return;
        // Map simple controls to the manager
        var g = manager.gravity; g.y = gravityY; manager.gravity = g;
        manager.groundRestitution = restitution;
        manager.groundFriction = friction;
        manager.airDamping = Mathf.Clamp01(1f - Mathf.Clamp01(linearDamping));
        manager.breakImpulseThreshold = Mathf.Max(0f, breakImpulse);
        manager.impactSpeedBreakThreshold = Mathf.Max(0f, impactSpeedThreshold);
        manager.initialDropHeight = dropHeight;
        manager.initialVelocity = initialVelocity;
    }
}
