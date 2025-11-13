using UnityEngine;
using System.Collections.Generic;

/// <summary>
/// Manages constrained rigid body dynamics simulation
/// Pure physics implementation - no Unity physics engine
/// Implements velocity-level Lagrange multiplier constraint solving
/// Reference: "Constrained Rigid Body Simulation" (ODE, Bullet physics)
/// </summary>
public class FractureManager : MonoBehaviour
{
    
    [Header("PhysX-like Controls")]
    [Tooltip("Break impulse threshold (N·s). Higher = harder to break. Roughly mass*impactSpeed.")]
    public float breakImpulseThreshold = 10f;    // Impulse to break (N·s). ~m*v. For m=1kg from 5m drop, v≈10m/s
    
    [Header("Integration Parameters (Advanced)")]
    public float timeStep = 0.02f;            // Fixed timestep (s)
    public int constraintIterations = 10;     // Gauss-Seidel iterations
    public float errorReductionParameter = 0.2f; // Baumgarte stabilization (Γ)
    
    [Header("External Forces")]
    [Tooltip("Gravity (m/s^2). Y should be negative for downward gravity.")]
    public Vector3 gravity = new Vector3(0, -9.81f, 0); // m/s²
    [Tooltip("Multiplicative velocity damping per step. 1=no damping, 0=full stop. Maps to 'linear damping' intuition.")]
    [Range(0.90f, 1.0f)] public float airDamping = 0.995f;         // Velocity damping
    
    [Header("Ground Collision")]
    public float groundY = 0f;                // Ground plane Y position
    [Tooltip("Coefficient of restitution (bounciness). 0=no bounce, 1=perfectly elastic")]
    [Range(0f,1f)] public float groundRestitution = 0.2f;    // Coefficient of restitution (less bouncy, more realistic)
    [Tooltip("Coulomb friction coefficient (0..1). Higher = more resistance to sliding")]
    [Range(0f,1f)] public float groundFriction = 0.6f;       // Friction coefficient (realistic for concrete/wood)
    [Tooltip("Minimum impact speed (m/s) required before constraints can break on contact.")]
    public float impactSpeedBreakThreshold = 2f;
    
    [Header("Initial Conditions")]
    [Tooltip("Height (m) to spawn above ground.")]
    public float initialDropHeight = 5f;
    [Tooltip("Initial linear velocity (m/s). Positive Y is upward, negative is downward.")]
    public Vector3 initialVelocity = Vector3.zero;
    
    // ========== Simulation State ==========
    public List<Constraint> constraints = new List<Constraint>();
    private Fragment[] fragments;
    private float accumulatedTime = 0f;
    private float simulationTime = 0f;  // Track total simulation time for warmup
    private bool hasFractured = false;   // Delay inter-fragment collisions until something breaks
    
    void Start()
    {
        // Wait for generator to create fragments
        Invoke("InitializeSimulation", 0.1f);
    }
    
  void InitializeSimulation()
{
    fragments = GetComponentsInChildren<Fragment>();
    if (fragments.Length == 0) return;

    Debug.Log($"Initializing {fragments.Length} fragments");

    // FIRST: Initialize all fragment physics states at their CURRENT positions
    // This ensures they start with zero velocity and proper orientation
    foreach (var fragment in fragments)
    {
        fragment.linearVelocity = Vector3.zero;
        fragment.angularVelocity = Vector3.zero;
        fragment.orientation = fragment.transform.rotation;
        fragment.force = Vector3.zero;
        fragment.torque = Vector3.zero;
    }

    // SECOND: Create constraints BEFORE moving (so distances are correct)
    CreateConstraintNetwork();

    // THIRD: Move the PARENT to drop height (fragments move with it as children)
    // This preserves all relative positions perfectly
    transform.position = new Vector3(transform.position.x, initialDropHeight, transform.position.z);

    // FOURTH: Add initial velocity if needed
    foreach (var fragment in fragments)
    {
        fragment.linearVelocity = initialVelocity;
    }

    Debug.Log($"Cube moved to drop height {initialDropHeight}m with {constraints.Count} constraints");
}


    
    /// <summary>
    /// Create constraints ONLY between face-to-face adjacent fragments
    /// Each fragment connects to 6 neighbors: left, right, up, down, forward, back
    /// </summary>
    void CreateConstraintNetwork()
    {
        constraints.Clear();
        
        for (int i = 0; i < fragments.Length; i++)
        {
            for (int j = i + 1; j < fragments.Length; j++)
            {
                Vector3 posA = fragments[i].transform.position;
                Vector3 posB = fragments[j].transform.position;
                Vector3 delta = posB - posA;
                float distance = delta.magnitude;
                
                float cubeSize = fragments[i].size.x; // Assuming cubic fragments
                
                // Check if fragments are face-to-face adjacent
                // Face-to-face means distance ≈ cubeSize AND aligned on ONE axis only
                bool isFaceAdjacent = false;
                
                // Tolerance for floating point comparison
                float tolerance = cubeSize * 0.1f;
                
                // Check X-axis alignment (left/right neighbors)
                if (Mathf.Abs(delta.x - cubeSize) < tolerance && 
                    Mathf.Abs(delta.y) < tolerance && 
                    Mathf.Abs(delta.z) < tolerance)
                {
                    isFaceAdjacent = true; // Right neighbor
                }
                // Check Y-axis alignment (up/down neighbors)
                else if (Mathf.Abs(delta.y - cubeSize) < tolerance && 
                         Mathf.Abs(delta.x) < tolerance && 
                         Mathf.Abs(delta.z) < tolerance)
                {
                    isFaceAdjacent = true; // Up neighbor
                }
                // Check Z-axis alignment (forward/back neighbors)
                else if (Mathf.Abs(delta.z - cubeSize) < tolerance && 
                         Mathf.Abs(delta.x) < tolerance && 
                         Mathf.Abs(delta.y) < tolerance)
                {
                    isFaceAdjacent = true; // Forward neighbor
                }
                
                if (isFaceAdjacent)
                {
                    Constraint c = new Constraint(fragments[i], fragments[j], 0f, breakImpulseThreshold);
                    
                    // Anchor points at CENTER of each fragment (default is Vector3.zero in local space)
                    // This means constraint pulls from center-to-center
                    c.localAnchorA = Vector3.zero;
                    c.localAnchorB = Vector3.zero;
                    
                    constraints.Add(c);
                    
                    // Debug first few constraints
                    if (constraints.Count <= 10)
                    {
                        Debug.Log($"Constraint {constraints.Count}: {fragments[i].name} <-> {fragments[j].name}, distance={distance:F3} (face-to-face, center-to-center)");
                    }
                }
            }
        }
        
        Debug.Log($"Created {constraints.Count} constraints - cube is already perfectly aligned!");
    }
    
    void Update()
    {
        if (fragments == null || fragments.Length == 0) return;
        
        // Accumulate time for fixed timestep integration
        accumulatedTime += Time.deltaTime;
        
        // Run physics at fixed timestep for stability
        while (accumulatedTime >= timeStep)
        {
            PhysicsStep(timeStep);
            accumulatedTime -= timeStep;
        }
    }
    
    /// <summary>
    /// Main physics simulation step
    /// Implements semi-implicit Euler integration with constraint solving
    /// </summary>
    void PhysicsStep(float dt)
    {
        // ========== Step 1: Apply External Forces ==========
        foreach (var fragment in fragments)
        {
            // Reset force and torque accumulators
            fragment.force = Vector3.zero;
            fragment.torque = Vector3.zero;
            
            // Apply gravity: F = mg
            fragment.AddForce(gravity * fragment.mass);
        }
        
        // ========== Step 2: Apply Constraint Forces ==========
        // Remove broken constraints
        for (int i = constraints.Count - 1; i >= 0; i--)
        {
            if (constraints[i].isBroken)
            {
                constraints.RemoveAt(i);
            }
        }
        
        // ========== Step 2: Apply Constraint Forces ==========
        // PASSIVE MODE: No forces applied - constraints do nothing during free-fall
        // Fragments fall under gravity only, staying together geometrically
        // Constraints only activate on ground impact to break and scatter
        
        foreach (var constraint in constraints)
        {
            constraint.ApplyConstraintForces(); // This does nothing (passive)
            constraint.UpdateVisualLine(); // Update visual representation
        }
        
        // ========== Step 3: Integrate Velocities (Semi-Implicit Euler) ==========
        foreach (var fragment in fragments)
        {
            // Linear: v = v + (F/m) * dt
            Vector3 acceleration = fragment.force / fragment.mass;
            fragment.linearVelocity += acceleration * dt;
            
            // Angular: ω = ω + (I^(-1) * τ) * dt
            fragment.UpdateInertiaTensor();
            Vector3 angularAcceleration = Matrix4x4Math.Multiply3x3(
                fragment.inertiaTensorWorldInverse,
                fragment.torque
            );
            fragment.angularVelocity += angularAcceleration * dt;
            
            // Apply air damping
            fragment.linearVelocity *= airDamping;
            fragment.angularVelocity *= airDamping;
        }
        
        // ========== Step 4: Integrate Positions & Rotations (no quaternions) ==========
        foreach (var fragment in fragments)
        {
            // Linear: x = x + v * dt
            fragment.transform.position += fragment.linearVelocity * dt;

            // Angular: integrate using axis-angle and apply via Transform.Rotate (world space)
            Vector3 w = fragment.angularVelocity;
            float wmag = w.magnitude;
            if (wmag > 1e-5f)
            {
                Vector3 axis = w / wmag;
                float angleDeg = wmag * dt * Mathf.Rad2Deg;
                fragment.transform.Rotate(axis, angleDeg, Space.World);
            }
        }
        
        // ========== Step 5: Collision Detection and Response ==========
        HandleGroundCollisions();
        HandleFragmentCollisions();
    }
    
    /// <summary>
    /// Handle collisions with ground plane
    /// Realistic physics: each fragment gets unique response based on impact point
    /// </summary>
    void HandleGroundCollisions()
    {
        foreach (var fragment in fragments)
        {
            float halfHeight = fragment.size.y * 0.5f;
            float penetration = groundY + halfHeight - fragment.transform.position.y;
            
            if (penetration > 0)
            {
                // Position correction
                fragment.transform.position += Vector3.up * penetration;
                
                // Calculate impact point (bottom of cube) using transform basis
                Vector3 impactPoint = fragment.transform.position - fragment.transform.up * halfHeight;
                
                // Get velocity at impact point (includes rotational component)
                Vector3 impactVelocity = fragment.GetVelocityAtPoint(impactPoint);
                Vector3 normal = Vector3.up;
                
                float velocityAlongNormal = Vector3.Dot(impactVelocity, normal);
                
                if (velocityAlongNormal < 0) // Moving into ground
                {
                    float impactSpeed = Mathf.Abs(velocityAlongNormal);
                    
                    // Check for high-velocity impact - break constraints
                    if (impactSpeed > impactSpeedBreakThreshold)
                    {
                        BreakConstraintsOnImpact(fragment, impactSpeed);
                    }
                    
                    // ========== REALISTIC COLLISION RESPONSE ==========
                    
                    // 1. Normal impulse with restitution
                    // Calculate impulse at impact point (includes inertia effects)
                    Vector3 r = impactPoint - fragment.transform.position;
                    Vector3 rCrossN = Vector3.Cross(r, normal);
                    Vector3 angularEffect = Matrix4x4Math.Multiply3x3(fragment.inertiaTensorWorldInverse, rCrossN);
                    float effectiveMass = 1f / fragment.mass + Vector3.Dot(Vector3.Cross(angularEffect, r), normal);
                    effectiveMass = 1f / effectiveMass;
                    
                    // Impulse magnitude: J = -(1+e) * v_n * m_eff
                    float normalImpulseMagnitude = -(1f + groundRestitution) * velocityAlongNormal * effectiveMass;
                    Vector3 normalImpulse = normal * normalImpulseMagnitude;
                    
                    // Apply impulse at impact point (creates rotation)
                    fragment.ApplyImpulseAtPoint(normalImpulse, impactPoint);
                    
                    // 2. Tangential friction
                    Vector3 tangentialVelocity = impactVelocity - velocityAlongNormal * normal;
                    float tangentialSpeed = tangentialVelocity.magnitude;
                    
                    if (tangentialSpeed > 0.01f)
                    {
                        Vector3 tangentDirection = tangentialVelocity / tangentialSpeed;
                        
                        // Coulomb friction: F_friction ≤ μ * F_normal
                        float maxFrictionImpulse = groundFriction * Mathf.Abs(normalImpulseMagnitude);
                        float frictionImpulse = Mathf.Min(tangentialSpeed * effectiveMass, maxFrictionImpulse);
                        
                        Vector3 frictionImpulseVector = -tangentDirection * frictionImpulse;
                        fragment.ApplyImpulseAtPoint(frictionImpulseVector, impactPoint);
                    }
                    
                    // 3. Add realistic scatter based on impact geometry
                    // Off-center impacts cause rotation
                    Vector3 centerOffset = impactPoint - fragment.transform.position;
                    centerOffset.y = 0; // Only horizontal offset matters for scatter
                    
                    if (centerOffset.sqrMagnitude > 0.001f)
                    {
                        // Corner/edge hit creates spin
                        Vector3 scatterTorque = Vector3.Cross(centerOffset.normalized, Vector3.up) * impactSpeed * 0.5f;
                        fragment.AddTorque(scatterTorque * fragment.mass);
                        
                        // Slight lateral impulse for realistic scatter
                        Vector3 scatterDirection = new Vector3(centerOffset.x, 0, centerOffset.z).normalized;
                        Vector3 scatterImpulse = scatterDirection * impactSpeed * 0.3f * fragment.mass;
                        fragment.ApplyImpulse(scatterImpulse);
                    }
                    
                    // 4. Random micro-perturbations for natural variation
                    // Real surfaces aren't perfectly flat
                    Vector3 randomTorque = new Vector3(
                        Random.Range(-1f, 1f),
                        Random.Range(-1f, 1f),
                        Random.Range(-1f, 1f)
                    ) * impactSpeed * 0.2f;
                    fragment.AddTorque(randomTorque);
                }
                
                // Continuous friction while resting on ground
                if (Mathf.Abs(fragment.transform.position.y - (groundY + halfHeight)) < 0.01f)
                {
                    // Rolling resistance
                    fragment.angularVelocity *= (1f - groundFriction * 0.1f);
                    
                    // Sliding friction
                    Vector3 horizontalVelocity = new Vector3(fragment.linearVelocity.x, 0, fragment.linearVelocity.z);
                    if (horizontalVelocity.sqrMagnitude > 0.001f)
                    {
                        fragment.linearVelocity.x *= (1f - groundFriction * 0.2f);
                        fragment.linearVelocity.z *= (1f - groundFriction * 0.2f);
                    }
                }
            }
        }
    }
    
    /// <summary>
    /// Handle collisions between fragments
    /// Realistic impulse resolution with rotation
    /// </summary>
    void HandleFragmentCollisions()
    {
        // Important: before any fracture, fragments are packed in a perfect grid.
        // Our simple sphere approximation would detect many false overlaps
        // (edges/corners) and push pieces apart. So we disable inter-fragment
        // collisions until at least one constraint has broken.
        if (!hasFractured) return;

        for (int i = 0; i < fragments.Length; i++)
        {
            for (int j = i + 1; j < fragments.Length; j++)
            {
                Fragment fragA = fragments[i];
                Fragment fragB = fragments[j];
                
                // Simple sphere collision detection (using bounding radius)
                Vector3 delta = fragB.transform.position - fragA.transform.position;
                float distance = delta.magnitude;
                float minDistance = (fragA.size.magnitude + fragB.size.magnitude) * 0.5f;
                
                if (distance < minDistance && distance > 0.001f)
                {
                    Vector3 normal = delta / distance;
                    float penetration = minDistance - distance;
                    
                    // Position correction (based on mass ratio for realism)
                    float totalMass = fragA.mass + fragB.mass;
                    float ratioA = fragB.mass / totalMass;
                    float ratioB = fragA.mass / totalMass;
                    
                    fragA.transform.position -= normal * (penetration * ratioA);
                    fragB.transform.position += normal * (penetration * ratioB);
                    
                    // Calculate collision point (on surface between fragments)
                    Vector3 collisionPoint = fragA.transform.position + normal * (fragA.size.magnitude * 0.5f);
                    
                    // Get velocities at collision point (includes rotation)
                    Vector3 velA = fragA.GetVelocityAtPoint(collisionPoint);
                    Vector3 velB = fragB.GetVelocityAtPoint(collisionPoint);
                    Vector3 relativeVelocity = velB - velA;
                    float velocityAlongNormal = Vector3.Dot(relativeVelocity, normal);
                    
                    if (velocityAlongNormal < 0) // Approaching
                    {
                        // Calculate effective mass at collision point
                        // Includes rotational inertia effects
                        Vector3 rA = collisionPoint - fragA.transform.position;
                        Vector3 rB = collisionPoint - fragB.transform.position;
                        
                        Vector3 rACrossN = Vector3.Cross(rA, normal);
                        Vector3 rBCrossN = Vector3.Cross(rB, normal);
                        
                        Vector3 angularA = Matrix4x4Math.Multiply3x3(fragA.inertiaTensorWorldInverse, rACrossN);
                        Vector3 angularB = Matrix4x4Math.Multiply3x3(fragB.inertiaTensorWorldInverse, rBCrossN);
                        
                        float invMassA = 1f / fragA.mass + Vector3.Dot(Vector3.Cross(angularA, rA), normal);
                        float invMassB = 1f / fragB.mass + Vector3.Dot(Vector3.Cross(angularB, rB), normal);
                        
                        // Impulse magnitude with restitution
                        float restitution = 0.4f; // Fragment-fragment collision is slightly bouncy
                        float impulseMagnitude = -(1f + restitution) * velocityAlongNormal / (invMassA + invMassB);
                        
                        Vector3 impulse = normal * impulseMagnitude;
                        
                        // Apply impulses at collision point (creates realistic rotation)
                        fragA.ApplyImpulseAtPoint(-impulse, collisionPoint);
                        fragB.ApplyImpulseAtPoint(impulse, collisionPoint);
                        
                        // Tangential friction for fragment-fragment contact
                        Vector3 tangentialVel = relativeVelocity - velocityAlongNormal * normal;
                        float tangentialSpeed = tangentialVel.magnitude;
                        
                        if (tangentialSpeed > 0.01f)
                        {
                            float friction = 0.3f; // Inter-fragment friction
                            Vector3 tangentDirection = tangentialVel / tangentialSpeed;
                            float frictionImpulse = Mathf.Min(tangentialSpeed / (invMassA + invMassB), friction * Mathf.Abs(impulseMagnitude));
                            
                            Vector3 frictionImpulseVector = -tangentDirection * frictionImpulse;
                            fragA.ApplyImpulseAtPoint(-frictionImpulseVector, collisionPoint);
                            fragB.ApplyImpulseAtPoint(frictionImpulseVector, collisionPoint);
                        }
                    }
                }
            }
        }
    }
    
    /// <summary>
    /// Break constraints connected to fragment on high-velocity impact
    /// More realistic scatter with rotation
    /// </summary>
    void BreakConstraintsOnImpact(Fragment fragment, float impactVelocity)
    {
        int brokenCount = 0;
        
        // Calculate impact impulse (momentum change)
        Vector3 impactImpulse = fragment.linearVelocity * fragment.mass;
        float impulseMagnitude = impactImpulse.magnitude;
        
        foreach (var constraint in constraints)
        {
            if (constraint.fragmentA == fragment || constraint.fragmentB == fragment)
            {
                if (!constraint.isBroken)
                {
                    // Store impact impulse in constraint for break reaction
                    constraint.impactImpulse = impactImpulse;
                    
                    // Break if impulse exceeds threshold
                    if (impulseMagnitude > constraint.breakImpulseThreshold)
                    {
                        constraint.Break();
                        brokenCount++;
                    }
                }
            }
        }
        
        if (brokenCount > 0)
        {
            hasFractured = true; // enable inter-fragment collisions from now on
            Debug.Log($"Impact! Velocity: {impactVelocity:F2} m/s, Impulse: {impulseMagnitude:F1}N⋅s, Broke {brokenCount} constraints");
        }
    }
    
    // ========== Quaternion Math (manual implementation) ==========
    
    Quaternion QuaternionMultiply(Quaternion a, Quaternion b)
    {
        return new Quaternion(
            a.w * b.x + a.x * b.w + a.y * b.z - a.z * b.y,
            a.w * b.y - a.x * b.z + a.y * b.w + a.z * b.x,
            a.w * b.z + a.x * b.y - a.y * b.x + a.z * b.w,
            a.w * b.w - a.x * b.x - a.y * b.y - a.z * b.z
        );
    }
    
    Quaternion ScaleQuaternion(Quaternion q, float scale)
    {
        return new Quaternion(q.x * scale, q.y * scale, q.z * scale, q.w * scale);
    }
    
    Quaternion AddQuaternions(Quaternion a, Quaternion b)
    {
        return new Quaternion(a.x + b.x, a.y + b.y, a.z + b.z, a.w + b.w);
    }
    
    Quaternion NormalizeQuaternion(Quaternion q)
    {
        float magnitude = Mathf.Sqrt(q.x * q.x + q.y * q.y + q.z * q.z + q.w * q.w);
        if (magnitude < 0.0001f) return Quaternion.identity;
        return new Quaternion(q.x / magnitude, q.y / magnitude, q.z / magnitude, q.w / magnitude);
    }
    
    // ========== Debug Visualization ==========
    
    void OnDrawGizmos()
    {
        // Draw ground plane
        Gizmos.color = new Color(0.5f, 0.5f, 0.5f, 0.5f);
        Vector3 center = new Vector3(0, groundY, 0);
        Gizmos.DrawWireCube(center, new Vector3(20, 0.1f, 20));
        
        // Note: Constraints are now drawn with LineRenderers (see Constraint.cs)
        // They will automatically disappear when broken
    }
}
