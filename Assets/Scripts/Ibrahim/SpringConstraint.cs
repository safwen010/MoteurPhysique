using UnityEngine;

namespace S7an
{
    /// <summary>
    /// Spring constraint implementation based on "Energized Rigid Body Fracture" paper
    /// Stores elastic energy and converts it to kinetic energy on fracture
    /// E = 1/2 * k * x² (elastic potential energy)
    /// Δv = √(2E/m) (velocity from energy conversion)
    /// </summary>
    [System.Serializable]
    public class SpringConstraint
    {
        [Header("Connected Fragments")]
        public RigidFragment fragmentA;
        public RigidFragment fragmentB;
        
        [Header("Spring Parameters")]
        [Tooltip("Spring stiffness constant k (N/m)")]
        public float springConstant = 2000f;
        
        [Tooltip("Rest length of spring (m)")]
        public float restLength = 0.4f;
        
        [Tooltip("Damping coefficient (Ns/m)")]
        public float dampingCoefficient = 10f;
        
        [Tooltip("Break threshold - impulse magnitude")]
        public float breakThreshold = 25f; // Lower threshold for easier breaking
        
        [Header("Energy System")]
        [Tooltip("Energy transfer coefficient (0-1 physical, >1 artistic)")]
        public float alpha = 1.0f;
        
        [Tooltip("Current stored elastic energy E = 1/2*k*x²")]
        public float storedEnergy = 0f;
        
        [Tooltip("Current spring deformation")]
        public float currentDeformation = 0f;
        
        [Tooltip("Current constraint impulse magnitude")]
        public float currentImpulse = 0f;
        
        [Header("State")]
        public bool isBroken = false;
        
        // Internal calculation variables
        private Vector3 lastForceA = Vector3.zero;
        private Vector3 lastTorqueA = Vector3.zero;
        private Vector3 lastForceB = Vector3.zero;
        private Vector3 lastTorqueB = Vector3.zero;
        
        public void Initialize(RigidFragment fragA, RigidFragment fragB, float energyTransferCoeff)
        {
            fragmentA = fragA;
            fragmentB = fragB;
            alpha = energyTransferCoeff;
            
            // Calculate rest length as initial distance
            Vector3 delta = fragB.transform.position - fragA.transform.position;
            restLength = delta.magnitude;
            
            isBroken = false;
            storedEnergy = 0f;
            currentDeformation = 0f;
        }
        
        /// <summary>
        /// Apply spring constraint forces between fragments
        /// Implements spring-damper system: F = -kx - cv
        /// </summary>
        public void ApplyConstraintForces(float deltaTime)
        {
            if (isBroken || fragmentA == null || fragmentB == null) return;
            
            Vector3 posA = fragmentA.transform.position;
            Vector3 posB = fragmentB.transform.position;
            
            Vector3 delta = posB - posA;
            float currentLength = delta.magnitude;
            
            if (currentLength < 0.001f) return;
            
            Vector3 direction = delta / currentLength;
            
            // Calculate deformation (positive = extension, negative = compression)
            float deformation = currentLength - restLength;
            currentDeformation = Mathf.Abs(deformation);
            
            // Spring force: F = -k * x
            float springForceMagnitude = -springConstant * deformation;
            
            // Damping force: F = -c * v_relative
            Vector3 velocityA = fragmentA.velocity;
            Vector3 velocityB = fragmentB.velocity;
            Vector3 relativeVelocity = velocityB - velocityA;
            float dampingForceMagnitude = -dampingCoefficient * Vector3.Dot(relativeVelocity, direction);
            
            // Total force
            float totalForceMagnitude = springForceMagnitude + dampingForceMagnitude;
            Vector3 forceVector = direction * totalForceMagnitude;
            
            // Apply forces to fragments
            fragmentA.AddForce(forceVector);
            fragmentB.AddForce(-forceVector);
            
            // Store forces for impulse calculation
            lastForceA = forceVector;
            lastForceB = -forceVector;
            
            // Calculate stored elastic energy: E = 1/2 * k * x²
            storedEnergy = 0.5f * springConstant * deformation * deformation;
            
            // Calculate current impulse (force * dt approximation)
            currentImpulse = Mathf.Abs(totalForceMagnitude) * deltaTime;
        }
        
        /// <summary>
        /// Check if constraint should break based on impulse threshold
        /// </summary>
        public bool ShouldBreak()
        {
            if (isBroken) return false;
            
            // Break if impulse exceeds threshold
            return currentImpulse > breakThreshold;
        }
        
        /// <summary>
        /// Apply fracture impulse based on stored elastic energy
        /// Implements the core algorithm from the research paper
        /// </summary>
        public void ApplyFractureImpulse()
        {
            if (isBroken || fragmentA == null || fragmentB == null) return;
            
            isBroken = true;
            
            Debug.Log($"Applying fracture impulse - Stored energy: {storedEnergy:F2} J, Deformation: {currentDeformation:F4} m");
            
            // Calculate impulse direction based on constraint forces and torques
            Vector3 impulseDirection = CalculateImpulseDirection();
            
            // Calculate effective masses for each fragment at contact point
            Vector3 contactPoint = (fragmentA.transform.position + fragmentB.transform.position) * 0.5f;
            float effectiveMassA = CalculateEffectiveMass(fragmentA, contactPoint, impulseDirection);
            float effectiveMassB = CalculateEffectiveMass(fragmentB, contactPoint, impulseDirection);
            
            // Energy distribution based on masses (from paper)
            float totalMass = effectiveMassA + effectiveMassB;
            float energyA = storedEnergy * alpha * (effectiveMassB / totalMass);
            float energyB = storedEnergy * alpha * (effectiveMassA / totalMass);
            
            // Calculate impulse magnitudes: μ = √(2αmE) from paper equation
            float impulseA = Mathf.Sqrt(2f * energyA * effectiveMassA);
            float impulseB = Mathf.Sqrt(2f * energyB * effectiveMassB);
            
            // Apply impulses in opposite directions
            Vector3 impulseVectorA = -impulseDirection * impulseA;
            Vector3 impulseVectorB = impulseDirection * impulseB;
            
            fragmentA.AddImpulseAtPoint(impulseVectorA, contactPoint);
            fragmentB.AddImpulseAtPoint(impulseVectorB, contactPoint);
            
            // Add rotational impulses for dynamic fracture effect
            Vector3 randomAngularA = new Vector3(
                Random.Range(-1f, 1f),
                Random.Range(-1f, 1f),
                Random.Range(-1f, 1f)
            ) * storedEnergy * 0.2f;
            
            Vector3 randomAngularB = new Vector3(
                Random.Range(-1f, 1f),
                Random.Range(-1f, 1f),
                Random.Range(-1f, 1f)
            ) * storedEnergy * 0.2f;
            
            fragmentA.AddAngularImpulse(randomAngularA);
            fragmentB.AddAngularImpulse(randomAngularB);
            
            Debug.Log($"Applied impulses - A: {impulseA:F2} N⋅s, B: {impulseB:F2} N⋅s, Direction: {impulseDirection}");
        }
        
        /// <summary>
        /// Calculate impulse direction based on forces and torques
        /// Implementation of direction calculation from research paper
        /// </summary>
        Vector3 CalculateImpulseDirection()
        {
            // Use the cross product of torque and force as in the paper
            // l̂ = -(τ × f) / ||τ × f||
            
            Vector3 force = lastForceA;
            Vector3 torque = lastTorqueA;
            
            if (force.magnitude < 0.001f)
            {
                // If no significant force, use separation direction
                Vector3 separation = (fragmentB.transform.position - fragmentA.transform.position).normalized;
                return separation;
            }
            
            if (torque.magnitude < 0.001f)
            {
                // If no significant torque, use force direction
                return force.normalized;
            }
            
            // Cross product for direction
            Vector3 crossProduct = Vector3.Cross(torque, force);
            
            if (crossProduct.magnitude < 0.001f)
            {
                // Parallel vectors, use perpendicular direction
                Vector3 perpendicular = Vector3.Cross(force, Vector3.up);
                if (perpendicular.magnitude < 0.001f)
                {
                    perpendicular = Vector3.Cross(force, Vector3.right);
                }
                return perpendicular.normalized;
            }
            
            return -crossProduct.normalized;
        }
        
        /// <summary>
        /// Calculate effective mass at contact point (includes rotational inertia)
        /// From paper: mG = (GM⁻¹Gᵀ)⁻¹
        /// </summary>
        float CalculateEffectiveMass(RigidFragment fragment, Vector3 contactPoint, Vector3 direction)
        {
            Vector3 leverArm = contactPoint - fragment.transform.position;
            Vector3 angularComponent = Vector3.Cross(leverArm, direction);
            
            // For simplicity, assume uniform box inertia
            // In full implementation, would use proper inertia tensor
            float linearMass = fragment.mass;
            float angularMass = 1f; // Simplified - should be based on inertia tensor
            
            // Effective mass combines linear and angular components
            return 1f / (1f / linearMass + angularComponent.sqrMagnitude / angularMass);
        }
        
        /// <summary>
        /// Get current constraint status for debugging
        /// </summary>
        public string GetStatus()
        {
            if (isBroken) return "BROKEN";
            
            return $"Energy: {storedEnergy:F2}J, Deform: {currentDeformation:F4}m, Impulse: {currentImpulse:F2}Ns";
        }
        
        /// <summary>
        /// Visualize constraint in Scene view
        /// </summary>
        public void DrawGizmos()
        {
            if (fragmentA == null || fragmentB == null) return;
            
            Vector3 posA = fragmentA.transform.position;
            Vector3 posB = fragmentB.transform.position;
            
            if (isBroken)
            {
                Gizmos.color = Color.red;
            }
            else
            {
                // Color based on stored energy
                float energyRatio = Mathf.Clamp01(storedEnergy / 10f);
                Gizmos.color = Color.Lerp(Color.green, Color.yellow, energyRatio);
            }
            
            Gizmos.DrawLine(posA, posB);
            
            // Draw energy level as sphere at midpoint
            if (!isBroken && storedEnergy > 0.1f)
            {
                Vector3 midpoint = (posA + posB) * 0.5f;
                float sphereSize = Mathf.Clamp(storedEnergy * 0.01f, 0.02f, 0.1f);
                Gizmos.DrawWireSphere(midpoint, sphereSize);
            }
        }
    }
}