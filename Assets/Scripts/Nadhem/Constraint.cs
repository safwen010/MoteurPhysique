using UnityEngine;

/// <summary>
/// Passive constraint that holds fragments in place
/// NO FORCES applied during free-fall - just gravity
/// Breaks on ground impact, releasing reaction force
/// </summary>
[System.Serializable]
public class Constraint
{
    // ========== Connected Fragments ==========
    public Fragment fragmentA;
    public Fragment fragmentB;
    
    // ========== Constraint Parameters ==========
    [Tooltip("Break impulse threshold (N·s). Higher = stronger bond; lower = breaks easier.")]
    public float breakImpulseThreshold = 500f;   // Impulse needed to break (N⋅s)
    public float initialCompressionForce = 100f; // Force to tighten cube at spawn
    
    // ========== Constraint State ==========
    public bool isBroken = false;
    public bool applyForces = false;             // TRUE = apply constraint forces, FALSE = passive (just gravity)
    public Vector3 impactImpulse = Vector3.zero; // Impulse to apply when breaking
    public float accumulatedDamage = 0f;         // Damage accumulation for progressive failure
    
    // ========== Attachment Points (body-local coordinates) ==========
    public Vector3 localAnchorA = Vector3.zero;
    public Vector3 localAnchorB = Vector3.zero;
    
    // ========== Visual Representation ==========
    private GameObject lineObject;
    private LineRenderer lineRenderer;
    
    // Constructor - creates passive constraint between two fragments
    public Constraint(Fragment a, Fragment b, float unused1, float breakThreshold)
    {
        fragmentA = a;
        fragmentB = b;
        breakImpulseThreshold = breakThreshold;
        
        // Initialize local anchors (center of each fragment)
        localAnchorA = Vector3.zero;
        localAnchorB = Vector3.zero;
        
        // Start passive - no forces applied
        applyForces = false;
        
        // Create visual line for this constraint
        CreateVisualLine();
    }
    
    /// <summary>
    /// Get world position of anchor point on fragment A
    /// </summary>
    public Vector3 GetWorldAnchorA()
    {
        // Use Transform to convert local anchor to world (no quaternion math here)
        return fragmentA.transform.TransformPoint(localAnchorA);
    }
    
    /// <summary>
    /// Get world position of anchor point on fragment B
    /// </summary>
    public Vector3 GetWorldAnchorB()
    {
        return fragmentB.transform.TransformPoint(localAnchorB);
    }
    
    /// <summary>
    /// Create a visual line to represent the constraint
    /// </summary>
    void CreateVisualLine()
    {
        if (fragmentA == null || fragmentB == null) return;
        
        // Create a new GameObject for the line
        lineObject = new GameObject("Constraint_Line");
        lineRenderer = lineObject.AddComponent<LineRenderer>();
        
        // Configure line appearance
        lineRenderer.startWidth = 0.02f;
        lineRenderer.endWidth = 0.02f;
        lineRenderer.material = new Material(Shader.Find("Sprites/Default"));
        lineRenderer.startColor = Color.yellow;
        lineRenderer.endColor = Color.yellow;
        lineRenderer.positionCount = 2;
        lineRenderer.useWorldSpace = true;
        
        // Update initial positions
        UpdateVisualLine();
    }
    
    /// <summary>
    /// Update the visual line positions
    /// </summary>
    public void UpdateVisualLine()
    {
        if (lineRenderer == null || isBroken) return;
        
        Vector3 posA = GetWorldAnchorA();
        Vector3 posB = GetWorldAnchorB();
        
        lineRenderer.SetPosition(0, posA);
        lineRenderer.SetPosition(1, posB);
        
        // Color based on stress (green = relaxed, red = near breaking)
        float stress = Mathf.Clamp01(
            Mathf.Max(
                impactImpulse.magnitude / breakImpulseThreshold,
                accumulatedDamage / Mathf.Max(1e-4f, breakImpulseThreshold)
            )
        );
        Color color = Color.Lerp(Color.green, Color.red, stress);
        lineRenderer.startColor = color;
        lineRenderer.endColor = color;
    }
    
    /// <summary>
    /// Apply initial compression force to tighten the cube
    /// Call this right after creation to pull fragments together
    /// </summary>
    public void ApplyInitialCompression(float dt)
    {
        if (isBroken) return;
        
        Vector3 posA = GetWorldAnchorA();
        Vector3 posB = GetWorldAnchorB();
        
        // Direction from A to B
        Vector3 delta = posB - posA;
        float distance = delta.magnitude;
        
        if (distance < 0.0001f) return;
        
        Vector3 direction = delta / distance;
        
        // Apply compression force to pull fragments together
        // This creates a tight, compressed initial state
        Vector3 compressionForce = direction * initialCompressionForce;
        
        // Apply force at anchor points (generates both linear and angular motion)
        fragmentA.AddForceAtPoint(compressionForce, posA);
        fragmentB.AddForceAtPoint(-compressionForce, posB);
    }
    
    /// <summary>
    /// PASSIVE constraint - NO forces applied
    /// Fragments fall together under gravity only
    /// This method does nothing during free-fall
    /// </summary>
    public void ApplyConstraintForces()
    {
        // DO NOTHING - constraints are passive until impact
        // Fragments stay together geometrically but no forces applied
        return;
    }
    
    /// <summary>
    /// Break the constraint and apply stored impulse
    /// Called when ground impact is detected
    /// </summary>
    public void Break()
    {
        if (isBroken) return;
        
        isBroken = true;
        
        // Destroy the visual line immediately
        if (lineObject != null)
        {
            Object.Destroy(lineObject);
            lineObject = null;
            lineRenderer = null;
        }
        
        Debug.Log($"Constraint broken! Impact impulse: {impactImpulse.magnitude:F1}N⋅s");
        
        // Get attachment points
        Vector3 posA = GetWorldAnchorA();
        Vector3 posB = GetWorldAnchorB();
        
        // Calculate direction from A to B
        Vector3 delta = posB - posA;
        if (delta.sqrMagnitude < 1e-6f) return;
        Vector3 direction = delta.normalized;
        
        // Apply impact impulse in opposite directions (reaction force)
        // This scatters fragments apart from the impact
        Vector3 scatterImpulseA = -direction * impactImpulse.magnitude * 0.5f;
        Vector3 scatterImpulseB = direction * impactImpulse.magnitude * 0.5f;
        
        fragmentA.ApplyImpulseAtPoint(scatterImpulseA, posA);
        fragmentB.ApplyImpulseAtPoint(scatterImpulseB, posB);
        
        // Add rotational chaos from break
        float torqueMagnitude = impactImpulse.magnitude * 0.1f;
        fragmentA.AddTorque(Random.onUnitSphere * torqueMagnitude);
        fragmentB.AddTorque(Random.onUnitSphere * torqueMagnitude);
    }

    /// <summary>
    /// Add damage progressively; break when threshold exceeded.
    /// </summary>
    public void AddDamage(float amount)
    {
        if (isBroken) return;
        accumulatedDamage += Mathf.Max(0f, amount);
        if (accumulatedDamage >= breakImpulseThreshold)
        {
            Break();
        }
    }
    
    /// <summary>
    /// Get compliance (not used in passive mode)
    /// </summary>
    public float GetCompliance()
    {
        return 0f; // Infinitely stiff (passive constraint)
    }
    
    /// <summary>
    /// Get current distance between fragments
    /// </summary>
    public float GetConstraintDistance()
    {
        Vector3 posA = GetWorldAnchorA();
        Vector3 posB = GetWorldAnchorB();
        return Vector3.Distance(posA, posB);
    }
}
