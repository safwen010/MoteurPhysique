using UnityEngine;

namespace ChainSim
{
    // Represents a rigid fragment used in the chain simulation. Lightweight, minimal API
    // required by ChainManager and ChainDistanceConstraint.
    public class Fragment : MonoBehaviour
    {
        [Header("Physical")]
        public float mass = 1f;
        public Vector3 size = Vector3.one;

        // Kinematic state
        [HideInInspector] public Vector3 linearVelocity;
        [HideInInspector] public Vector3 angularVelocity; // world-space angular velocity (rad/s)
        [HideInInspector] public Quaternion orientation;

        // Forces/torques accumulated per-step
        [HideInInspector] public Vector3 force;
        [HideInInspector] public Vector3 torque;

        // Cached world-space inverse inertia tensor (top-left 3x3)
        [HideInInspector] public Matrix4x4 inertiaTensorWorldInverse = Matrix4x4.identity;

        void Awake()
        {
            orientation = transform.rotation;
        }

        // Recompute world-space inverse inertia tensor for a box-shaped fragment
        public void UpdateInertiaTensor()
        {
            float m = Mathf.Max(1e-6f, mass);
            // Box inertia in body frame: Ixx = 1/12 m (h^2 + d^2)
            float sx = Mathf.Abs(size.x);
            float sy = Mathf.Abs(size.y);
            float sz = Mathf.Abs(size.z);
            float Ixx = (1f / 12f) * m * (sy * sy + sz * sz);
            float Iyy = (1f / 12f) * m * (sx * sx + sz * sz);
            float Izz = (1f / 12f) * m * (sx * sx + sy * sy);

            // Build body-space inverse inertia (3x3)
            float invIxx = Ixx > 0f ? 1f / Ixx : 0f;
            float invIyy = Iyy > 0f ? 1f / Iyy : 0f;
            float invIzz = Izz > 0f ? 1f / Izz : 0f;

            Matrix4x4 IbInv = Matrix4x4.zero;
            IbInv.m00 = invIxx; IbInv.m11 = invIyy; IbInv.m22 = invIzz; IbInv.m33 = 1f;

            // Rotate to world: Iworld_inv = R * IbInv * R^T
            Matrix4x4 R = Matrix4x4.Rotate(orientation);
            Matrix4x4 Rt = R.transpose;
            inertiaTensorWorldInverse = Matrix4x4Math.Multiply3x3(R, Matrix4x4Math.Multiply3x3(IbInv, Rt));
        }

        // Return linear velocity at a world-space point p (v + omega x r)
        public Vector3 GetVelocityAtPoint(Vector3 p)
        {
            Vector3 r = p - transform.position;
            return linearVelocity + Vector3.Cross(angularVelocity, r);
        }

        // Apply linear impulse at center
        public void ApplyImpulse(Vector3 impulse)
        {
            linearVelocity += impulse / Mathf.Max(1e-6f, mass);
        }

        // Add a force (accumulates until integrated by the manager)
        public void AddForce(Vector3 f)
        {
            force += f;
        }

        // Apply impulse at world-space point p (affects linear and angular vel)
        public void ApplyImpulseAtPoint(Vector3 impulse, Vector3 p)
        {
            // Linear
            ApplyImpulse(impulse);

            // Angular delta = I^{-1} * (r x J)
            Vector3 r = p - transform.position;
            Vector3 torqueImpulse = Vector3.Cross(r, impulse);
            Vector3 deltaW = Matrix4x4Math.Multiply3x3(inertiaTensorWorldInverse, torqueImpulse);
            angularVelocity += deltaW;
        }
    }
}
