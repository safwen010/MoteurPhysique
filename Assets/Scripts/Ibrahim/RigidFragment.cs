using UnityEngine;

namespace S7an
{
    /// <summary>
    /// Individual rigid fragment with manual physics simulation
    /// No Unity physics components - all calculations done manually
    /// </summary>
    public class RigidFragment : MonoBehaviour
    {
        [Header("Physical Properties")]
        public float mass = 1.0f;
        public Vector3 size = Vector3.one;
        public Vector3 centerOfMass = Vector3.zero;
        
        [Header("Motion State")]
        public Vector3 velocity = Vector3.zero;
        public Vector3 angularVelocity = Vector3.zero;
        public Vector3 acceleration = Vector3.zero;
        public Vector3 angularAcceleration = Vector3.zero;
        
        // Forces and torques accumulated this frame
        private Vector3 accumulatedForce = Vector3.zero;
        private Vector3 accumulatedTorque = Vector3.zero;
        
        // Inertia tensor (for box shape)
        private Vector3 inertiaTensor;
        
        // Damping parameters (set by main controller)
        private float velocityDampingFactor = 0.999f;
        private float angularVelocityDampingFactor = 0.998f;
        
        public void Initialize(float fragmentMass, Vector3 fragmentSize)
        {
            mass = fragmentMass;
            size = fragmentSize;
            centerOfMass = Vector3.zero; // Center of box
            
            // Calculate inertia tensor for box shape
            // I = (m/12) * (w² + h²) for each axis
            float m = mass;
            float w = size.x;
            float h = size.y;
            float d = size.z;
            
            inertiaTensor = new Vector3(
                m * (h * h + d * d) / 12f, // Ixx
                m * (w * w + d * d) / 12f, // Iyy  
                m * (w * w + h * h) / 12f  // Izz
            );
            
            Debug.Log($"Fragment initialized - Mass: {mass}, Size: {size}, Inertia: {inertiaTensor}");
        }
        
        /// <summary>
        /// Set damping parameters from main controller
        /// </summary>
        public void SetDampingParameters(float velocityDamping, float angularVelocityDamping)
        {
            velocityDampingFactor = velocityDamping;
            angularVelocityDampingFactor = angularVelocityDamping;
        }
        
        /// <summary>
        /// Add force to be applied this physics step
        /// </summary>
        public void AddForce(Vector3 force)
        {
            accumulatedForce += force;
        }
        
        /// <summary>
        /// Add torque to be applied this physics step
        /// </summary>
        public void AddTorque(Vector3 torque)
        {
            accumulatedTorque += torque;
        }
        
        /// <summary>
        /// Add force at a specific world point (creates torque)
        /// </summary>
        public void AddForceAtPoint(Vector3 force, Vector3 worldPoint)
        {
            AddForce(force);
            
            // Calculate torque: τ = r × F
            Vector3 leverArm = worldPoint - transform.position;
            Vector3 torque = Vector3.Cross(leverArm, force);
            AddTorque(torque);
        }
        
        /// <summary>
        /// Apply impulse instantly (immediate velocity change)
        /// </summary>
        public void AddImpulse(Vector3 impulse)
        {
            velocity += impulse / mass;
        }
        
        /// <summary>
        /// Apply angular impulse instantly
        /// </summary>
        public void AddAngularImpulse(Vector3 angularImpulse)
        {
            // ω = I⁻¹ * L (angular momentum to angular velocity)
            Vector3 deltaAngularVel = new Vector3(
                angularImpulse.x / inertiaTensor.x,
                angularImpulse.y / inertiaTensor.y,
                angularImpulse.z / inertiaTensor.z
            );
            
            angularVelocity += deltaAngularVel;
            
            Debug.Log($"Angular impulse: {angularImpulse.magnitude:F2} -> Angular velocity change: {deltaAngularVel.magnitude:F2} -> Total: {angularVelocity.magnitude:F2}");
        }
        
        /// <summary>
        /// Apply impulse at a specific world point
        /// </summary>
        public void AddImpulseAtPoint(Vector3 impulse, Vector3 worldPoint)
        {
            AddImpulse(impulse);
            
            // Calculate angular impulse
            Vector3 leverArm = worldPoint - transform.position;
            Vector3 angularImpulse = Vector3.Cross(leverArm, impulse);
            AddAngularImpulse(angularImpulse);
        }
        
        /// <summary>
        /// Get velocity at a specific world point
        /// v = v_center + ω × r
        /// </summary>
        public Vector3 GetVelocityAtPoint(Vector3 worldPoint)
        {
            Vector3 leverArm = worldPoint - transform.position;
            return velocity + Vector3.Cross(angularVelocity, leverArm);
        }
        
        /// <summary>
        /// Integrate motion for one physics step
        /// Manual integration using Euler method
        /// </summary>
        public void IntegrateMotion(float deltaTime)
        {
            // Linear motion: F = ma
            acceleration = accumulatedForce / mass;
            velocity += acceleration * deltaTime;
            
            // Update position using manual matrix transformation
            Vector3 newPosition = transform.position + velocity * deltaTime;
            
            // Angular motion: τ = Iα
            angularAcceleration = new Vector3(
                accumulatedTorque.x / inertiaTensor.x,
                accumulatedTorque.y / inertiaTensor.y,
                accumulatedTorque.z / inertiaTensor.z
            );
            
            angularVelocity += angularAcceleration * deltaTime;
            
            // Update rotation using manual matrix calculation
            Quaternion currentRotation = transform.rotation;
            
            // Convert angular velocity to quaternion change
            // dq/dt = 0.5 * ω * q (quaternion derivative)
            float angleThisFrame = angularVelocity.magnitude * deltaTime;
            if (angleThisFrame > 0.001f)
            {
                Vector3 axis = angularVelocity.normalized;
                Quaternion deltaRotation = Quaternion.AngleAxis(angleThisFrame * Mathf.Rad2Deg, axis);
                currentRotation = deltaRotation * currentRotation;
                
                // Debug rotation integration
                if (angularVelocity.magnitude > 1f) // Only log significant rotations
                {
                    Debug.Log($"Rotation: AngVel={angularVelocity.magnitude:F2} rad/s, Angle this frame={angleThisFrame * Mathf.Rad2Deg:F2}°, Axis={axis}");
                }
            }
            
            // Apply transformations using our manual matrix system
            ApplyManualTransform(newPosition, currentRotation);
            
            // Clear accumulated forces for next frame
            accumulatedForce = Vector3.zero;
            accumulatedTorque = Vector3.zero;
            
            // Apply damping using parameters from inspector
            velocity *= velocityDampingFactor;
            angularVelocity *= angularVelocityDampingFactor;
        }
        
        /// <summary>
        /// Apply transformation using manual 4x4 matrices (no Unity built-ins)
        /// </summary>
        void ApplyManualTransform(Vector3 position, Quaternion rotation)
        {
            // Create manual transformation matrix
            Matrix4x4 transformMatrix = CreateManualTRSMatrix(position, rotation, transform.localScale);
            
            // Extract position and rotation from our manual matrix
            Vector3 matrixPosition = ExtractPosition(transformMatrix);
            Quaternion matrixRotation = ExtractRotation(transformMatrix);
            
            // Apply to transform
            transform.position = matrixPosition;
            transform.rotation = matrixRotation;
        }
        
        /// <summary>
        /// Create TRS matrix manually (Translation * Rotation * Scale)
        /// </summary>
        Matrix4x4 CreateManualTRSMatrix(Vector3 translation, Quaternion rotation, Vector3 scale)
        {
            // Manual translation matrix
            Matrix4x4 T = Matrix4x4.identity;
            T.m03 = translation.x;
            T.m13 = translation.y;
            T.m23 = translation.z;
            
            // Manual rotation matrix from quaternion
            Matrix4x4 R = CreateRotationMatrix(rotation);
            
            // Manual scale matrix
            Matrix4x4 S = Matrix4x4.identity;
            S.m00 = scale.x;
            S.m11 = scale.y;
            S.m22 = scale.z;
            
            // Combine: TRS
            return T * R * S;
        }
        
        /// <summary>
        /// Create rotation matrix from quaternion manually
        /// </summary>
        Matrix4x4 CreateRotationMatrix(Quaternion q)
        {
            Matrix4x4 m = Matrix4x4.identity;
            
            float x = q.x, y = q.y, z = q.z, w = q.w;
            float x2 = x + x, y2 = y + y, z2 = z + z;
            float xx = x * x2, xy = x * y2, xz = x * z2;
            float yy = y * y2, yz = y * z2, zz = z * z2;
            float wx = w * x2, wy = w * y2, wz = w * z2;
            
            m.m00 = 1f - (yy + zz);
            m.m01 = xy - wz;
            m.m02 = xz + wy;
            
            m.m10 = xy + wz;
            m.m11 = 1f - (xx + zz);
            m.m12 = yz - wx;
            
            m.m20 = xz - wy;
            m.m21 = yz + wx;
            m.m22 = 1f - (xx + yy);
            
            return m;
        }
        
        /// <summary>
        /// Extract position from transformation matrix
        /// </summary>
        Vector3 ExtractPosition(Matrix4x4 matrix)
        {
            return new Vector3(matrix.m03, matrix.m13, matrix.m23);
        }
        
        /// <summary>
        /// Extract rotation from transformation matrix
        /// </summary>
        Quaternion ExtractRotation(Matrix4x4 matrix)
        {
            Vector3 forward = new Vector3(matrix.m02, matrix.m12, matrix.m22);
            Vector3 upwards = new Vector3(matrix.m01, matrix.m11, matrix.m21);
            return Quaternion.LookRotation(forward, upwards);
        }
        
        void OnDrawGizmos()
        {
            // Visualize center of mass
            Gizmos.color = Color.blue;
            Gizmos.DrawWireSphere(transform.position + centerOfMass, 0.05f);
            
            // Visualize velocity
            if (velocity.magnitude > 0.1f)
            {
                Gizmos.color = Color.green;
                Gizmos.DrawRay(transform.position, velocity * 0.5f);
            }
            
            // Visualize angular velocity
            if (angularVelocity.magnitude > 0.1f)
            {
                Gizmos.color = Color.cyan;
                Gizmos.DrawRay(transform.position, angularVelocity * 0.2f);
            }
        }
    }
}