using UnityEngine;

/// <summary>
/// Rigid body fragment with complete 6-DOF physics
/// No Unity physics - pure manual simulation
/// Implements: F = ma, τ = Iα, constraint-based dynamics
/// </summary>
public class Fragment : MonoBehaviour
{
    // ========== Physical Properties ==========
    public float mass = 1f; // Mass in kg
    public Vector3 size = Vector3.one * 0.5f; // Dimensions for inertia calculation
    
    // ========== State Variables ==========
    // Position (stored in transform.position)
    // Orientation (stored as quaternion)
    public Quaternion orientation = Quaternion.identity;
    
    // Linear motion
    public Vector3 linearVelocity = Vector3.zero; // m/s
    public Vector3 linearMomentum = Vector3.zero; // p = mv
    
    // Angular motion
    public Vector3 angularVelocity = Vector3.zero; // rad/s (in world space)
    public Vector3 angularMomentum = Vector3.zero; // L = Iω
    
    // ========== Force and Torque Accumulators ==========
    public Vector3 force = Vector3.zero; // Total force (N)
    public Vector3 torque = Vector3.zero; // Total torque (N⋅m)
    
    // ========== Inertia Tensor ==========
    // For a uniform density cube: I = (1/12) * m * (h² + d²) for each axis
    // Stored in body-local coordinates (diagonal matrix for box)
    public Vector3 inertiaTensorLocal = Vector3.one;
    
    // World-space inertia tensor (rotated by orientation)
    // I_world = R * I_local * R^T
    public float[,] inertiaTensorWorld = new float[3, 3];
    public float[,] inertiaTensorWorldInverse = new float[3, 3];
    [HideInInspector]
public Vector3 initialLocalPosition; // store the local position at Start()

    
    void Start()
    {
        initialLocalPosition = transform.localPosition;

        // Calculate inertia tensor for a box (cube)
        // I_x = (1/12) * m * (y² + z²)
        // I_y = (1/12) * m * (x² + z²)
        // I_z = (1/12) * m * (x² + y²)
        float x = size.x;
        float y = size.y;
        float z = size.z;
        
        inertiaTensorLocal = new Vector3(
            (1f / 12f) * mass * (y * y + z * z),
            (1f / 12f) * mass * (x * x + z * z),
            (1f / 12f) * mass * (x * x + y * y)
        );
        
        // Initialize orientation from transform
        orientation = transform.rotation;
        
        UpdateInertiaTensor();
    }
    
    /// <summary>
    /// Update world-space inertia tensor: I_world = R * I_local * R^T
    /// </summary>
    public void UpdateInertiaTensor()
    {
        // Build 3x3 rotation matrix from Transform basis (no quaternion math)
        Vector3 r = transform.right;
        Vector3 u = transform.up;
        Vector3 f = transform.forward;
        float[,] R = new float[3,3];
        // Columns are basis vectors: [r u f]
        R[0,0] = r.x; R[1,0] = r.y; R[2,0] = r.z;
        R[0,1] = u.x; R[1,1] = u.y; R[2,1] = u.z;
        R[0,2] = f.x; R[1,2] = f.y; R[2,2] = f.z;
        float[,] RT = Matrix4x4Math.Transpose3x3(R);
        
        // Create diagonal inertia matrix in local space
        float[,] I_local = new float[3, 3];
        I_local[0, 0] = inertiaTensorLocal.x;
        I_local[1, 1] = inertiaTensorLocal.y;
        I_local[2, 2] = inertiaTensorLocal.z;
        
        // I_world = R * I_local * R^T
        float[,] temp = Matrix4x4Math.Multiply3x3(R, I_local);
        inertiaTensorWorld = Matrix4x4Math.Multiply3x3(temp, RT);
        
        // Calculate inverse for angular acceleration calculation
        inertiaTensorWorldInverse = InvertInertiaTensor(inertiaTensorWorld);
    }
    
    /// <summary>
    /// Invert the 3x3 inertia tensor (simplified for symmetric matrices)
    /// </summary>
    private float[,] InvertInertiaTensor(float[,] I)
    {
        float[,] inv = new float[3, 3];
        
        // Calculate determinant
        float det = I[0, 0] * (I[1, 1] * I[2, 2] - I[1, 2] * I[2, 1])
                  - I[0, 1] * (I[1, 0] * I[2, 2] - I[1, 2] * I[2, 0])
                  + I[0, 2] * (I[1, 0] * I[2, 1] - I[1, 1] * I[2, 0]);
        
        if (Mathf.Abs(det) < 1e-10f)
        {
            // Singular matrix, return identity
            inv[0, 0] = inv[1, 1] = inv[2, 2] = 1f;
            return inv;
        }
        
        float invDet = 1f / det;
        
        // Calculate adjugate matrix and divide by determinant
        inv[0, 0] = (I[1, 1] * I[2, 2] - I[1, 2] * I[2, 1]) * invDet;
        inv[0, 1] = (I[0, 2] * I[2, 1] - I[0, 1] * I[2, 2]) * invDet;
        inv[0, 2] = (I[0, 1] * I[1, 2] - I[0, 2] * I[1, 1]) * invDet;
        inv[1, 0] = (I[1, 2] * I[2, 0] - I[1, 0] * I[2, 2]) * invDet;
        inv[1, 1] = (I[0, 0] * I[2, 2] - I[0, 2] * I[2, 0]) * invDet;
        inv[1, 2] = (I[0, 2] * I[1, 0] - I[0, 0] * I[1, 2]) * invDet;
        inv[2, 0] = (I[1, 0] * I[2, 1] - I[1, 1] * I[2, 0]) * invDet;
        inv[2, 1] = (I[0, 1] * I[2, 0] - I[0, 0] * I[2, 1]) * invDet;
        inv[2, 2] = (I[0, 0] * I[1, 1] - I[0, 1] * I[1, 0]) * invDet;
        
        return inv;
    }
    
    /// <summary>
    /// Apply force at center of mass (no torque)
    /// </summary>
    public void AddForce(Vector3 f)
    {
        force += f;
    }
    
    /// <summary>
    /// Apply force at a point in world space
    /// Generates both force and torque: τ = r × F
    /// </summary>
    public void AddForceAtPoint(Vector3 f, Vector3 worldPoint)
    {
        force += f;
        
        // Calculate torque: τ = r × F
        Vector3 r = worldPoint - transform.position;
        torque += Vector3.Cross(r, f);
    }
    
    /// <summary>
    /// Apply torque directly
    /// </summary>
    public void AddTorque(Vector3 t)
    {
        torque += t;
    }
    
    /// <summary>
    /// Apply impulse (instantaneous velocity change)
    /// Δv = J / m
    /// </summary>
    public void ApplyImpulse(Vector3 impulse)
    {
        linearVelocity += impulse / mass;
    }
    
    /// <summary>
    /// Apply angular impulse at a point
    /// </summary>
    public void ApplyImpulseAtPoint(Vector3 impulse, Vector3 worldPoint)
    {
        // Linear impulse
        linearVelocity += impulse / mass;
        
        // Angular impulse: ΔL = r × J
        Vector3 r = worldPoint - transform.position;
        Vector3 angularImpulse = Vector3.Cross(r, impulse);
        
        // Δω = I^(-1) * ΔL
        angularVelocity += Matrix4x4Math.Multiply3x3(inertiaTensorWorldInverse, angularImpulse);
    }
    
    /// <summary>
    /// Get velocity at a point in world space
    /// v_point = v_com + ω × r
    /// </summary>
    public Vector3 GetVelocityAtPoint(Vector3 worldPoint)
    {
        Vector3 r = worldPoint - transform.position;
        return linearVelocity + Vector3.Cross(angularVelocity, r);
    }
    
    /// <summary>
    /// Calculate kinetic energy
    /// KE = (1/2)mv² + (1/2)ω·(I·ω)
    /// </summary>
    public float GetKineticEnergy()
    {
        float translational = 0.5f * mass * linearVelocity.sqrMagnitude;
        
        Vector3 L = Matrix4x4Math.Multiply3x3(inertiaTensorWorld, angularVelocity);
        float rotational = 0.5f * Vector3.Dot(angularVelocity, L);
        
        return translational + rotational;
    }
}
