using UnityEngine;

namespace ChainSim
{
    /// <summary>
    /// Analytic distance constraint between two Fragment anchor points (no PhysX).
    /// Solves at velocity level with ERP/CFM-style stabilization and applies impulses
    /// to both linear and angular velocity. Also tracks tension to allow breaking.
    /// </summary>
    public class ChainDistanceConstraint
    {
        public Fragment A;
        public Fragment B;
        public Vector3 localA = Vector3.zero; // anchor in A's local space
        public Vector3 localB = Vector3.zero; // anchor in B's local space
        public float restLength;

        // Solver parameters
        public float erp = 0.2f;   // error reduction (Baumgarte)
        public float cfm = 0.0f;   // compliance (softness)
    public float breakForce = 500f; // Newtons (peak tension to fail)
    public float damage = 0f;       // accumulated damage surrogate
    public float damageGain = 0.5f; // scales how fast tension accumulates into damage
    public float damageDecay = 0.98f; // per-solve decay (0..1)

        // State
        public bool broken = false;
        public float lastTension = 0f; // Newtons

        public ChainDistanceConstraint(Fragment a, Fragment b, float restLen)
        {
            A = a; B = b; restLength = restLen;
            localA = Vector3.zero;
            localB = Vector3.zero;
        }

        public void Solve(float dt)
        {
            if (broken) return;

            // World anchors
            Vector3 pA = A.transform.TransformPoint(localA);
            Vector3 pB = B.transform.TransformPoint(localB);

            Vector3 rA = pA - A.transform.position;
            Vector3 rB = pB - B.transform.position;

            Vector3 delta = pB - pA;
            float dist = delta.magnitude;
            if (dist < 1e-6f) return;
            Vector3 n = delta / dist;

            // position error
            float C = dist - restLength;

            // velocities at anchors
            Vector3 vA = A.GetVelocityAtPoint(pA);
            Vector3 vB = B.GetVelocityAtPoint(pB);
            float relVn = Vector3.Dot(vB - vA, n);

            // Effective mass along the constraint axis (includes inertia)
            Vector3 rAXn = Vector3.Cross(rA, n);
            Vector3 rBXn = Vector3.Cross(rB, n);
            Vector3 AngA = Matrix4x4Math.Multiply3x3(A.inertiaTensorWorldInverse, rAXn);
            Vector3 AngB = Matrix4x4Math.Multiply3x3(B.inertiaTensorWorldInverse, rBXn);
            float invMeff = 1f / A.mass + 1f / B.mass + Vector3.Dot(Vector3.Cross(AngA, rA) + Vector3.Cross(AngB, rB), n) + cfm;
            float meff = 1f / Mathf.Max(invMeff, 1e-6f);

            // ERP drives positional error toward zero; relVn damps velocity error
            float bTerm = (erp * C) / Mathf.Max(dt, 1e-5f);
            float jn = -(relVn + bTerm) * meff; // scalar impulse along n
            Vector3 J = n * jn;

            // Apply impulses
            A.ApplyImpulseAtPoint(-J, pA);
            B.ApplyImpulseAtPoint(J, pB);

            // Tension estimate (force = |impulse|/dt)
            lastTension = Mathf.Abs(jn) / Mathf.Max(dt, 1e-5f);

            // Progressive damage: accumulate with slight decay
            damage = damage * Mathf.Clamp01(damageDecay) + lastTension * dt * Mathf.Max(0f, damageGain);

            if (lastTension >= breakForce || damage >= breakForce)
            {
                broken = true;
            }
        }
    }
}
