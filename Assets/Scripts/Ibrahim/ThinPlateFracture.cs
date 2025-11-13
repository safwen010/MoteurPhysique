using UnityEngine;
using System.Collections.Generic;

namespace S7an
{
    /// <summary>
    /// Implementation of "Energized Rigid Body Fracture" thin plate example
    /// Creates a pre-scored plate that fractures on impact with sphere
    /// Based on the research paper by Li, Andrews, Jones, and Bargteil (2018)
    /// </summary>
    public class ThinPlateFracture : MonoBehaviour
    {
        [Header("Plate Configuration")]
        [Tooltip("Number of fragments along X axis")]
        public int fragmentsX = 5;
        
        [Tooltip("Number of fragments along Z axis")]
        public int fragmentsZ = 5;
        
        [Tooltip("Size of each fragment")]
        public Vector3 fragmentSize = new Vector3(0.4f, 0.1f, 0.4f);
        
        [Tooltip("Mass of each fragment")]
        public float fragmentMass = 1.0f;
        
        [Tooltip("Initial height above sphere")]
        public float dropHeight = 3.0f;
        
        [Header("Physics Parameters")]
        [Tooltip("Gravity vector")]
        public Vector3 gravity = new Vector3(0, -9.81f, 0);
        
        [Tooltip("Time step for physics simulation")]
        public float timeStep = 0.016f;
        
        [Header("Sphere Obstacle")]
        [Tooltip("Sphere center position")]
        public Vector3 sphereCenter = new Vector3(0, 1, 0);
        
        [Tooltip("Sphere radius")]
        public float sphereRadius = 1.0f;
        
        [Header("Ground Plane")]
        [Tooltip("Ground Y position (height level)")]
        public float groundY = 0f;
        
        [Tooltip("Ground plane size")]
        public Vector2 groundSize = new Vector2(20f, 20f);
        
        [Tooltip("Show visible ground plane")]
        public bool showGroundPlane = true;
        
        [Header("Spring Constraint Physics")]
        [Tooltip("Spring stiffness constant k (N/m) - F = kx")]
        [Range(100f, 10000f)]
        public float springConstant = 2000f;
        
        [Tooltip("Damping coefficient c (Ns/m) - F = cv")]
        [Range(1f, 100f)]
        public float dampingCoefficient = 10f;
        
        [Tooltip("Break threshold - impulse magnitude to fracture")]
        [Range(5f, 100f)]
        public float breakThreshold = 25f;
        
        [Tooltip("Energy transfer coefficient α (0-1 physical, >1 artistic)")]
        [Range(0f, 2f)]
        public float alpha = 1.0f;
        
        [Header("Collision Physics")]
        [Tooltip("Sphere collision restitution (0=plastic, 1=elastic)")]
        [Range(0f, 1.5f)]
        public float sphereRestitution = 0.4f;
        
        [Tooltip("Ground collision restitution (usually lower than sphere)")]
        [Range(0f, 1f)]
        public float groundRestitution = 0.2f;
        
        [Tooltip("Friction coefficient for horizontal sliding")]
        [Range(0f, 1f)]
        public float frictionCoefficient = 0.3f;
        
        [Tooltip("Angular damping factor (0=no damping, 1=full stop)")]
        [Range(0f, 1f)]
        public float angularDamping = 0.05f;
        
        [Header("Material Properties")]
        [Tooltip("Fragment surface roughness (affects scattering)")]
        [Range(0f, 1f)]
        public float surfaceRoughness = 0.3f;
        
        [Tooltip("Impact energy multiplier for constraint breaking")]
        [Range(0f, 1f)]
        public float shockTransmission = 0.1f;
        
        [Tooltip("Random scatter strength for realistic collisions")]
        [Range(0f, 2f)]
        public float scatterStrength = 0.8f;
        
        [Header("Damping and Stability")]
        [Tooltip("General velocity damping per frame (0.999 = light, 0.99 = heavy)")]
        [Range(0f, 1f)]
        public float velocityDamping = 0.999f;
        
        [Tooltip("Angular velocity damping per frame")]
        [Range(0f, 1f)]
        public float angularVelocityDamping = 0.998f;
        
        [Tooltip("Ground impact angular damping multiplier")]
        [Range(0f, 1f)]
        public float groundAngularDamping = 0.95f;
        
        [Header("Advanced Physics")]
        [Tooltip("Minimum velocity for cube settling (m/s)")]
        [Range(0.1f, 2f)]
        public float settlingVelocityThreshold = 0.5f;
        
        [Tooltip("Minimum angular velocity for cube settling (rad/s)")]
        [Range(0.1f, 5f)]
        public float settlingAngularThreshold = 1.0f;
        
        [Tooltip("Rotational impulse scaling factor")]
        [Range(0f, 1f)]
        public float rotationalImpulseScale = 0.1f;
        
        [Header("Debug Information")]
        [Tooltip("Show physics equation values in console")]
        public bool showDebugInfo = false;
        
        [Header("Real-time Physics Equations")]
        [Space(10)]
        [TextArea(3, 6)]
        public string physicsEquations = 
            "Spring Force: F = -kx - cv\n" +
            "Elastic Energy: E = ½kx²\n" +
            "Impulse: Δv = √(2E/m)\n" +
            "Collision: v' = -e·v\n" +
            "Friction: f = μN\n" +
            "Damping: v' = v·d";
        
        // Runtime data
        private List<RigidFragment> fragments = new List<RigidFragment>();
        private List<SpringConstraint> constraints = new List<SpringConstraint>();
        private GameObject sphereObject;
        private GameObject groundObject;
        private bool simulationStarted = false;
        
        void Start()
        {
            CreatePlateFragments();
            CreateSpringConstraints();
            CreateSphereVisual();
            CreateGroundVisual();
            
            // Auto-start simulation
            simulationStarted = true;
            
            Debug.Log($"Created {fragments.Count} fragments with {constraints.Count} constraints");
            Debug.Log("Simulation auto-started! Plate should start falling.");
        }
        
        void FixedUpdate()
        {
            if (!simulationStarted) return;
            
            SimulationStep(timeStep);
        }
        
        [ContextMenu("Start Simulation")]
        public void StartSimulation()
        {
            simulationStarted = true;
            Debug.Log("Thin plate fracture simulation started!");
        }
        
        [ContextMenu("Reset Simulation")]
        public void ResetSimulation()
        {
            simulationStarted = false;
            
            // Clear existing fragments
            foreach (var fragment in fragments)
            {
                if (fragment.gameObject != null)
                    DestroyImmediate(fragment.gameObject);
            }
            
            // Clear existing visuals
            if (sphereObject != null)
                DestroyImmediate(sphereObject);
            if (groundObject != null)
                DestroyImmediate(groundObject);
            
            fragments.Clear();
            constraints.Clear();
            
            // Recreate everything
            CreatePlateFragments();
            CreateSpringConstraints();
            CreateSphereVisual();
            CreateGroundVisual();
            
            Debug.Log("Simulation reset!");
        }
        
        [ContextMenu("Update Physics Parameters")]
        public void UpdatePhysicsParameters()
        {
            // Update all constraint parameters in real-time
            foreach (var constraint in constraints)
            {
                if (!constraint.isBroken)
                {
                    constraint.springConstant = springConstant;
                    constraint.dampingCoefficient = dampingCoefficient;
                    constraint.breakThreshold = breakThreshold;
                    constraint.alpha = alpha;
                }
            }
            
            Debug.Log("Physics parameters updated for all active constraints!");
        }
        
        [ContextMenu("Test Rotation")]
        public void TestRotation()
        {
            // Apply strong rotation to all fragments to test if rotation works
            foreach (var fragment in fragments)
            {
                fragment.angularVelocity = new Vector3(5f, 5f, 5f); // 5 rad/s around all axes
                Debug.Log($"Applied test rotation to {fragment.name}: {fragment.angularVelocity}");
            }
        }
        
        void CreatePlateFragments()
        {
            float totalWidth = fragmentsX * fragmentSize.x;
            float totalDepth = fragmentsZ * fragmentSize.z;
            
            Vector3 startPos = transform.position + new Vector3(-totalWidth * 0.5f + fragmentSize.x * 0.5f, 
                                                               dropHeight, 
                                                               -totalDepth * 0.5f + fragmentSize.z * 0.5f);
            
            for (int z = 0; z < fragmentsZ; z++)
            {
                for (int x = 0; x < fragmentsX; x++)
                {
                    Vector3 pos = startPos + new Vector3(x * fragmentSize.x, 0, z * fragmentSize.z);
                    CreateFragment(pos, x, z);
                }
            }
        }
        
        void CreateFragment(Vector3 position, int x, int z)
        {
            GameObject go = GameObject.CreatePrimitive(PrimitiveType.Cube);
            go.name = $"Fragment_{x}_{z}";
            go.transform.parent = transform;
            go.transform.position = position;
            go.transform.localScale = fragmentSize;
            
            // Remove Unity collider - we'll use our own physics
            Collider col = go.GetComponent<Collider>();
            if (col) DestroyImmediate(col);
            
            // Add our rigid fragment component
            RigidFragment fragment = go.AddComponent<RigidFragment>();
            fragment.Initialize(fragmentMass, fragmentSize);
            
            // Unique color for each fragment based on grid position
            MeshRenderer renderer = go.GetComponent<MeshRenderer>();
            if (renderer)
            {
                Material mat = new Material(Shader.Find("Universal Render Pipeline/Lit"));
                
                // Create unique color based on position with more variety
                float hue = ((float)(x + z * fragmentsX) / (fragmentsX * fragmentsZ)) * 360f;
                Color fragmentColor = Color.HSVToRGB(hue / 360f, 0.8f, 0.6f);
                
                // Alternative: Rainbow gradient across the plate
                // Color fragmentColor = new Color(
                //     0.3f + 0.7f * (float)x / (fragmentsX - 1),
                //     0.3f + 0.7f * (float)z / (fragmentsZ - 1),
                //     0.5f + 0.5f * Mathf.Sin((float)(x + z) * 0.5f)
                // );
                
                mat.SetColor("_BaseColor", fragmentColor);
                mat.SetFloat("_Metallic", 0.1f);
                mat.SetFloat("_Smoothness", 0.3f);
                renderer.material = mat;
            }
            
            fragments.Add(fragment);
        }
        
        void CreateSpringConstraints()
        {
            // Create constraints between adjacent fragments
            for (int z = 0; z < fragmentsZ; z++)
            {
                for (int x = 0; x < fragmentsX; x++)
                {
                    int currentIndex = z * fragmentsX + x;
                    RigidFragment current = fragments[currentIndex];
                    
                    // Right neighbor
                    if (x < fragmentsX - 1)
                    {
                        int rightIndex = z * fragmentsX + (x + 1);
                        RigidFragment right = fragments[rightIndex];
                        CreateConstraint(current, right);
                    }
                    
                    // Bottom neighbor
                    if (z < fragmentsZ - 1)
                    {
                        int bottomIndex = (z + 1) * fragmentsX + x;
                        RigidFragment bottom = fragments[bottomIndex];
                        CreateConstraint(current, bottom);
                    }
                }
            }
        }
        
        void CreateConstraint(RigidFragment fragmentA, RigidFragment fragmentB)
        {
            SpringConstraint constraint = new SpringConstraint();
            constraint.Initialize(fragmentA, fragmentB, alpha);
            
            // Apply inspector parameters to constraint
            constraint.springConstant = springConstant;
            constraint.dampingCoefficient = dampingCoefficient;
            constraint.breakThreshold = breakThreshold;
            constraint.alpha = alpha;
            
            constraints.Add(constraint);
        }
        
        void CreateSphereVisual()
        {
            sphereObject = GameObject.CreatePrimitive(PrimitiveType.Sphere);
            sphereObject.name = "Sphere Obstacle";
            sphereObject.transform.position = sphereCenter;
            sphereObject.transform.localScale = Vector3.one * (sphereRadius * 2);
            
            // Remove collider
            Collider col = sphereObject.GetComponent<Collider>();
            if (col) DestroyImmediate(col);
            
            // Dark red URP lit sphere
            MeshRenderer renderer = sphereObject.GetComponent<MeshRenderer>();
            if (renderer)
            {
                Material mat = new Material(Shader.Find("Universal Render Pipeline/Lit"));
                mat.SetColor("_BaseColor", new Color(0.6f, 0.05f, 0.05f)); // Dark red
                mat.SetFloat("_Metallic", 0.2f);
                mat.SetFloat("_Smoothness", 0.5f);
                renderer.material = mat;
            }
        }
        
        void CreateGroundVisual()
        {
            if (!showGroundPlane) return;
            
            // Create ground plane using a scaled cube (thinner than default)
            groundObject = GameObject.CreatePrimitive(PrimitiveType.Cube);
            groundObject.name = "Ground Plane";
            
            // Position and scale the ground
            groundObject.transform.position = new Vector3(0, groundY - 0.05f, 0); // Slightly below ground level
            groundObject.transform.localScale = new Vector3(groundSize.x, 0.1f, groundSize.y);
            
            // Remove collider (we handle ground collision manually)
            Collider col = groundObject.GetComponent<Collider>();
            if (col) DestroyImmediate(col);
            
            // Create ground material - gray concrete look
            MeshRenderer renderer = groundObject.GetComponent<MeshRenderer>();
            if (renderer)
            {
                Material mat = new Material(Shader.Find("Universal Render Pipeline/Lit"));
                mat.SetColor("_BaseColor", new Color(0.3f, 0.3f, 0.35f)); // Gray concrete
                mat.SetFloat("_Metallic", 0.0f); // No metal reflection
                mat.SetFloat("_Smoothness", 0.2f); // Rough surface
                renderer.material = mat;
            }
            
            Debug.Log($"Ground plane created at Y={groundY}, Size={groundSize}");
        }
        
        void SimulationStep(float dt)
        {
            // 1. Apply external forces (gravity)
            foreach (var fragment in fragments)
            {
                fragment.AddForce(gravity * fragment.mass);
            }
            
            // 2. Apply constraint forces and check for fracture
            for (int i = constraints.Count - 1; i >= 0; i--)
            {
                SpringConstraint constraint = constraints[i];
                
                if (!constraint.isBroken)
                {
                    constraint.ApplyConstraintForces(dt);
                    
                    // Check for fracture
                    if (constraint.ShouldBreak())
                    {
                        constraint.ApplyFractureImpulse();
                        constraints.RemoveAt(i);
                        Debug.Log($"Constraint broken! Stored energy: {constraint.storedEnergy:F2} J");
                    }
                }
            }
            
            // 3. Update fragment damping parameters from inspector
            foreach (var fragment in fragments)
            {
                fragment.SetDampingParameters(velocityDamping, angularVelocityDamping);
            }
            
            // 4. Integrate motion
            foreach (var fragment in fragments)
            {
                fragment.IntegrateMotion(timeStep);
            }
            
            // 4. Handle collisions with sphere
            foreach (var fragment in fragments)
            {
                HandleSphereCollision(fragment);
            }
            
            // 5. Handle ground collision
            foreach (var fragment in fragments)
            {
                HandleGroundCollision(fragment);
            }
        }
        
        void HandleSphereCollision(RigidFragment fragment)
        {
            Vector3 fragmentPos = fragment.transform.position;
            Vector3 toFragment = fragmentPos - sphereCenter;
            float distance = toFragment.magnitude;
            
            // Check if fragment intersects sphere (considering fragment size)
            float fragmentRadius = Mathf.Max(fragmentSize.x, fragmentSize.y, fragmentSize.z) * 0.5f;
            
            if (distance < sphereRadius + fragmentRadius)
            {
                // Collision response with realistic physics
                Vector3 normal = toFragment.normalized;
                float penetration = (sphereRadius + fragmentRadius) - distance;
                
                // Find the actual contact point on the fragment surface
                Vector3 contactPointOnFragment = FindContactPointOnCube(fragment, sphereCenter);
                
                // Separate fragments from sphere
                fragment.transform.position += normal * penetration;
                
                // Calculate impact velocity at the actual contact point
                Vector3 velocityAtContact = fragment.GetVelocityAtPoint(contactPointOnFragment);
                float impactSpeed = Vector3.Dot(velocityAtContact, normal);
                
                if (impactSpeed < 0) // Moving towards sphere
                {
                    float impactEnergy = 0.5f * fragment.mass * impactSpeed * impactSpeed;
                    
                    // Calculate collision impulse
                    Vector3 normalImpulse = -(1 + sphereRestitution) * impactSpeed * fragment.mass * normal;
                    
                    // Apply impulse at the actual contact point for realistic rotation
                    fragment.AddImpulseAtPoint(normalImpulse, contactPointOnFragment);
                    
                    // Calculate tangential component based on collision geometry
                    Vector3 leverArm = contactPointOnFragment - fragment.transform.position;
                    Vector3 tangentialDirection = Vector3.Cross(normal, leverArm).normalized;
                    
                    if (tangentialDirection.magnitude > 0.1f)
                    {
                        // Add tangential impulse based on collision angle and surface roughness
                        float tangentialMagnitude = normalImpulse.magnitude * surfaceRoughness * 0.3f;
                        Vector3 tangentialImpulse = tangentialDirection * tangentialMagnitude;
                        fragment.AddImpulseAtPoint(tangentialImpulse, contactPointOnFragment);
                    }
                    
                    // Add controlled angular impulse based on impact location
                    Vector3 rotationAxis = Vector3.Cross(leverArm, normal).normalized;
                    float rotationMagnitude = impactEnergy * rotationalImpulseScale * leverArm.magnitude * 10f; // Increased for visibility
                    Vector3 angularImpulse = rotationAxis * rotationMagnitude;
                    fragment.AddAngularImpulse(angularImpulse);
                    
                    Debug.Log($"Angular impulse applied: {angularImpulse.magnitude:F2}, Lever arm: {leverArm.magnitude:F2}, Rotation scale: {rotationalImpulseScale}");
                    
                    // Add some random scatter for realism (reduced since we have proper contact point)
                    Vector3 randomScatter = new Vector3(
                        Random.Range(-1f, 1f),
                        Random.Range(-1f, 1f),
                        Random.Range(-1f, 1f)
                    ) * scatterStrength * normalImpulse.magnitude * 0.1f;
                    fragment.AddImpulse(randomScatter);
                    
                    // Break nearby constraints due to impact shock
                    BreakConstraintsNearFragment(fragment, impactEnergy * shockTransmission);
                    
                    // Add immediate angular velocity for guaranteed visible rotation
                    Vector3 immediateRotation = rotationAxis * (impactEnergy * 0.5f);
                    fragment.angularVelocity += immediateRotation;
                    
                    Debug.Log($"Sphere impact! Speed: {Mathf.Abs(impactSpeed):F1}m/s, Angular velocity: {fragment.angularVelocity.magnitude:F2} rad/s, Lever: {leverArm.magnitude:F2}m");
                }
            }
        }
        
        /// <summary>
        /// Find the closest point on the cube surface to the sphere center
        /// This determines the actual collision point for realistic rotation
        /// </summary>
        Vector3 FindContactPointOnCube(RigidFragment fragment, Vector3 spherePos)
        {
            // Transform sphere position to fragment's local space
            Vector3 localSpherePos = fragment.transform.InverseTransformPoint(spherePos);
            
            // Cube extends from -size/2 to +size/2 in local space
            Vector3 halfSize = fragmentSize * 0.5f;
            
            // Clamp sphere position to cube bounds to find closest surface point
            Vector3 closestLocalPoint = new Vector3(
                Mathf.Clamp(localSpherePos.x, -halfSize.x, halfSize.x),
                Mathf.Clamp(localSpherePos.y, -halfSize.y, halfSize.y),
                Mathf.Clamp(localSpherePos.z, -halfSize.z, halfSize.z)
            );
            
            // If sphere center is inside cube, project to nearest face
            if (Mathf.Approximately(closestLocalPoint.x, localSpherePos.x) && 
                Mathf.Approximately(closestLocalPoint.y, localSpherePos.y) && 
                Mathf.Approximately(closestLocalPoint.z, localSpherePos.z))
            {
                // Find which face is closest
                float[] distances = {
                    halfSize.x - Mathf.Abs(localSpherePos.x), // X faces
                    halfSize.y - Mathf.Abs(localSpherePos.y), // Y faces  
                    halfSize.z - Mathf.Abs(localSpherePos.z)  // Z faces
                };
                
                int closestFace = 0;
                for (int i = 1; i < 3; i++)
                {
                    if (distances[i] < distances[closestFace])
                        closestFace = i;
                }
                
                // Project to the closest face
                switch (closestFace)
                {
                    case 0: // X face
                        closestLocalPoint.x = localSpherePos.x > 0 ? halfSize.x : -halfSize.x;
                        break;
                    case 1: // Y face
                        closestLocalPoint.y = localSpherePos.y > 0 ? halfSize.y : -halfSize.y;
                        break;
                    case 2: // Z face
                        closestLocalPoint.z = localSpherePos.z > 0 ? halfSize.z : -halfSize.z;
                        break;
                }
            }
            
            // Transform back to world space
            return fragment.transform.TransformPoint(closestLocalPoint);
        }
        
        void HandleGroundCollision(RigidFragment fragment)
        {
            // Use the inspector ground Y parameter
            float fragmentBottom = fragment.transform.position.y - fragmentSize.y * 0.5f;
            
            if (fragmentBottom < groundY)
            {
                // Separate
                Vector3 pos = fragment.transform.position;
                pos.y = groundY + fragmentSize.y * 0.5f;
                fragment.transform.position = pos;
                
                // Apply impulse if moving downward
                if (fragment.velocity.y < 0)
                {
                    float impactSpeed = Mathf.Abs(fragment.velocity.y);
                    
                    // Normal impulse using inspector restitution
                    Vector3 normalImpulse = Vector3.up * (1 + groundRestitution) * impactSpeed * fragment.mass;
                    fragment.AddImpulse(normalImpulse);
                    
                    // Add friction to horizontal movement using inspector friction
                    Vector3 horizontalVelocity = new Vector3(fragment.velocity.x, 0, fragment.velocity.z);
                    Vector3 frictionImpulse = -horizontalVelocity * fragment.mass * frictionCoefficient;
                    fragment.AddImpulse(frictionImpulse);
                    
                    // Add angular damping when hitting ground using inspector parameter
                    fragment.angularVelocity *= groundAngularDamping;
                }
            }
        }
        
        /// <summary>
        /// Break constraints connected to a fragment based on impact energy
        /// Simulates stress propagation through the structure
        /// </summary>
        void BreakConstraintsNearFragment(RigidFragment impactFragment, float shockEnergy)
        {
            List<SpringConstraint> constraintsToBreak = new List<SpringConstraint>();
            
            foreach (var constraint in constraints)
            {
                if (constraint.isBroken) continue;
                
                // Check if this constraint is connected to the impact fragment
                bool isConnected = (constraint.fragmentA == impactFragment || constraint.fragmentB == impactFragment);
                
                if (isConnected)
                {
                    // Apply shock energy to the constraint
                    float additionalStress = shockEnergy / constraint.springConstant;
                    constraint.currentDeformation += additionalStress;
                    
                    // Check if this additional stress causes fracture
                    if (constraint.ShouldBreak())
                    {
                        constraintsToBreak.Add(constraint);
                    }
                }
                else
                {
                    // Check nearby constraints (stress propagation)
                    Vector3 constraintCenter = (constraint.fragmentA.transform.position + constraint.fragmentB.transform.position) * 0.5f;
                    float distanceToImpact = Vector3.Distance(constraintCenter, impactFragment.transform.position);
                    
                    // Stress decreases with distance
                    if (distanceToImpact < fragmentSize.x * 2f) // Within 2 fragment widths
                    {
                        float stressFalloff = 1f / (1f + distanceToImpact);
                        float propagatedStress = shockEnergy * stressFalloff * 0.3f; // 30% propagation
                        
                        constraint.currentDeformation += propagatedStress / constraint.springConstant;
                        
                        if (constraint.ShouldBreak())
                        {
                            constraintsToBreak.Add(constraint);
                        }
                    }
                }
            }
            
            // Break all marked constraints
            foreach (var constraint in constraintsToBreak)
            {
                constraint.ApplyFractureImpulse();
                constraints.Remove(constraint);
                Debug.Log($"Constraint broken by impact shock! Energy: {constraint.storedEnergy:F2}J");
            }
        }
        
        void OnDrawGizmos()
        {
            // Draw sphere
            Gizmos.color = Color.red;
            Gizmos.DrawWireSphere(sphereCenter, sphereRadius);
            
            // Draw constraints
            Gizmos.color = Color.yellow;
            foreach (var constraint in constraints)
            {
                if (!constraint.isBroken && constraint.fragmentA != null && constraint.fragmentB != null)
                {
                    Gizmos.DrawLine(constraint.fragmentA.transform.position, 
                                   constraint.fragmentB.transform.position);
                }
            }
        }
    }
}