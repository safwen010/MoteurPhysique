using UnityEngine;
using System.Collections.Generic;

namespace ChainSim
{
    /// <summary>
    /// Runs a full custom physics simulation for a chain of Fragment links connected by distance constraints.
    /// No PhysX is used. Integrates linear+angular motion, solves constraints iteratively, handles ground contact,
    /// and applies a downward pull on the bottom anchor until a link breaks, after which the chain wiggles realistically.
    /// </summary>
    public class ChainManager : MonoBehaviour
    {
        [Header("References")]
        public ChainGenerator generator; // auto-detected if null

        [Header("Time")] public float timeStep = 0.016f; // 60 Hz

        [Header("Physics")]
        public Vector3 gravity = new Vector3(0, -9.81f, 0);
        [Range(0.90f, 1.0f)] public float airDamping = 0.995f;
        public float groundY = 0f;
        [Range(0f,1f)] public float groundRestitution = 0.2f;
        [Range(0f,1f)] public float groundFriction = 0.6f;
        [Range(0f,1f)] public float rollingResistance = 0.12f;

        [Header("Constraints")]
        public float restLengthScale = 1.0f; // multiply link height to set rope rest length
        public float erp = 0.2f; // stabilization
        public float cfm = 0.0f; // compliance
    public float breakForce = 250f; // N, per link (lower default to ensure break under pull)
    [Range(0f,2f)] public float damageGain = 0.6f; // scales accumulated damage gain
    [Range(0.9f,1f)] public float damageDecay = 0.98f; // per-solve decay
    public bool biasBreakAtBottom = true;
    [Range(0.1f,1f)] public float bottomBreakMultiplier = 0.6f; // bottom joint weaker
    public bool bottomBreaksFirst = true; // temporarily reinforce other joints until bottom breaks
    [Range(1f,10f)] public float othersPreBreakMultiplier = 3f;
        public int solverIterations = 6;

        [Header("Pulling")]
    public float pullForce = 200f; // N downward on bottom anchor (initial)
    public float pullRampTime = 1.0f; // seconds to ramp to initial pullForce
    public bool forceBottomBreak = true; // auto-increase pull until bottom snaps
    public float pullForceMax = 800f; // cap when forcing bottom break
    public float bottomBreakTimeout = 1.5f; // seconds to reach max while forcing
    public bool debugTension = false; // logs bottom joint tension

        [Header("Oscillation")]
        // Apply a horizontal oscillation force to the chain (for swinging)
        public bool enableOscillation = false;
        public Vector3 oscillationDirection = Vector3.right; // world-space direction
        public float oscillationAmplitude = 50f; // peak force (N)
        public float oscillationFrequency = 1f; // Hz
        [Range(0f, 1f)] public float oscillationAlpha = 1f; // blend 0..1 of oscillation influence
        public bool applyOscillationToAll = false; // if true apply to every fragment; otherwise only bottom anchor

        [Header("Stabilization")]
        // Controls how quickly fragments try to return to an upright (vertical) orientation.
        [Range(0f, 1f)] public float returnToVerticalAlpha = 0.5f; // 0: no correction, 1: full correction
        public float returnToVerticalStiffness = 12f; // torque scale applied per-radian of tilt
        public bool stabilizeRotation = true; // enable/disable this stabilization

    private List<ChainDistanceConstraint> constraints = new List<ChainDistanceConstraint>();
    private ChainDistanceConstraint bottomJoint;
    private Dictionary<ChainDistanceConstraint, float> normalBreakForces = new Dictionary<ChainDistanceConstraint, float>();
        private Fragment[] all;
        private float accumulator = 0f;
        private bool anyBroken = false;
    private bool preBreakReinforced = false;
    private bool pullDisabledAfterBreak = false;

        void Start()
        {
            if (generator == null) TryGetComponent(out generator);
            if (generator == null) { Debug.LogError("ChainManager needs ChainGenerator on the same GameObject."); enabled = false; return; }

            generator.Generate();
            BuildConstraints();
            CacheAllFragments();

            // initialize states
            foreach (var f in all)
            {
                f.linearVelocity = Vector3.zero; f.angularVelocity = Vector3.zero;
                f.orientation = f.transform.rotation; f.UpdateInertiaTensor();
                f.force = Vector3.zero; f.torque = Vector3.zero;
            }
        }

        void CacheAllFragments()
        {
            var list = new List<Fragment>();
            if (generator.topAnchor) list.Add(generator.topAnchor);
            list.AddRange(generator.links);
            if (generator.bottomAnchor) list.Add(generator.bottomAnchor);
            all = list.ToArray();
        }

        void BuildConstraints()
        {
            constraints.Clear();
            normalBreakForces.Clear(); bottomJoint = null; preBreakReinforced = false;
            var links = generator.links;
            float baseRest = generator.linkSize.y * restLengthScale + generator.gap;

            // Top anchor to first link
            if (links.Count > 0)
            {
                var c0 = new ChainDistanceConstraint(generator.topAnchor, links[0], baseRest);
                SetupConstraint(c0);
                constraints.Add(c0);
            }
            // Links chain
            for (int i = 0; i < links.Count - 1; i++)
            {
                var c = new ChainDistanceConstraint(links[i], links[i+1], baseRest);
                SetupConstraint(c);
                constraints.Add(c);
            }
            // Last link to bottom anchor
            if (links.Count > 0)
            {
                var cN = new ChainDistanceConstraint(links[links.Count - 1], generator.bottomAnchor, baseRest);
                SetupConstraint(cN);
                if (biasBreakAtBottom) cN.breakForce *= bottomBreakMultiplier;
                constraints.Add(cN);
                bottomJoint = cN;
            }

            // Save normal break forces and optionally reinforce others before bottom breaks
            foreach (var c in constraints)
            {
                normalBreakForces[c] = c.breakForce;
            }
            if (bottomBreaksFirst && bottomJoint != null)
            {
                foreach (var c in constraints)
                {
                    if (c == bottomJoint) continue;
                    c.breakForce *= othersPreBreakMultiplier;
                }
                preBreakReinforced = true;
            }
        }

        void SetupConstraint(ChainDistanceConstraint c)
        {
            c.erp = erp; c.cfm = cfm; c.breakForce = breakForce;
            c.damageGain = damageGain; c.damageDecay = damageDecay;
            // Use center-to-center anchors (already local zero). Could offset to ends for more realism.
            c.localA = Vector3.zero; c.localB = Vector3.zero;
        }

        void Update()
        {
            accumulator += Time.deltaTime;
            while (accumulator >= timeStep)
            {
                Step(timeStep);
                accumulator -= timeStep;
            }
        }

        void Step(float dt)
        {
            // 1. External forces
            foreach (var f in all)
            {
                f.force = gravity * f.mass; f.torque = Vector3.zero;
            }

            // Top anchor fixed: zero velocities and keep position
            var top = generator.topAnchor;
            if (top != null)
            {
                top.linearVelocity = Vector3.zero; top.angularVelocity = Vector3.zero;
                // Keep it where it was placed
            }

            // Pull bottom anchor downward (ramps up)
            var bottom = generator.bottomAnchor;
            if (bottom != null)
            {
                if (!pullDisabledAfterBreak)
                {
                    float tRamp = Mathf.Clamp01(Time.timeSinceLevelLoad / Mathf.Max(1e-3f, pullRampTime));
                    float F = pullForce * tRamp;
                    if (forceBottomBreak && (bottomJoint != null) && !bottomJoint.broken)
                    {
                        float tForce = Mathf.Clamp01(Time.timeSinceLevelLoad / Mathf.Max(0.1f, bottomBreakTimeout));
                        float Fextra = Mathf.Lerp(0f, Mathf.Max(0f, pullForceMax - pullForce), tForce);
                        F += Fextra;
                    }
                    bottom.AddForce(new Vector3(0, -F, 0));

                    // Optional horizontal oscillation force (sinusoidal)
                    if (enableOscillation && oscillationAlpha > 0f)
                    {
                        // compute instantaneous oscillation force (world-space)
                        float osc = Mathf.Sin(2f * Mathf.PI * oscillationFrequency * Time.timeSinceLevelLoad);
                        Vector3 hForce = oscillationDirection.normalized * (oscillationAmplitude * osc * Mathf.Clamp01(oscillationAlpha));

                        if (applyOscillationToAll)
                        {
                            // apply to each fragment except top anchor
                            foreach (var f in all)
                            {
                                if (f == generator.topAnchor) continue;
                                f.AddForce(hForce);
                            }
                        }
                        else
                        {
                            bottom.AddForce(hForce);
                        }
                    }
                }
            }

            // 2. Integrate velocities (semi-implicit)
            foreach (var f in all)
            {
                // Skip top anchor linear integration to keep fixed in place
                if (f == top) continue;
                Vector3 a = f.force / f.mass; f.linearVelocity += a * dt;
                f.UpdateInertiaTensor();
                Vector3 alpha = Matrix4x4Math.Multiply3x3(f.inertiaTensorWorldInverse, f.torque);
                f.angularVelocity += alpha * dt;
                f.linearVelocity *= airDamping; f.angularVelocity *= airDamping;
            }

            // 3. Solve constraints iteratively
            for (int it = 0; it < solverIterations; it++)
            {
                foreach (var c in constraints)
                {
                    if (!c.broken) c.Solve(dt);
                }
            }

            // Optional stabilization: apply corrective torque towards upright (vertical)
            if (stabilizeRotation && returnToVerticalAlpha > 0f)
            {
                float alpha = Mathf.Clamp01(returnToVerticalAlpha);
                float k = returnToVerticalStiffness * alpha;
                for (int i = 0; i < all.Length; i++)
                {
                    var f = all[i];
                    if (f == generator.topAnchor) continue; // top anchor is fixed

                    // current up vector in world space
                    Vector3 currUp = f.transform.up;
                    Vector3 desiredUp = Vector3.up;

                    // axis-angle between currUp and desiredUp
                    Vector3 axis = Vector3.Cross(currUp, desiredUp);
                    float sinA = axis.magnitude;
                    if (sinA < 1e-6f) continue;
                    float cosA = Mathf.Clamp(Vector3.Dot(currUp, desiredUp), -1f, 1f);
                    float angle = Mathf.Atan2(sinA, cosA); // angle in radians
                    Vector3 axisDir = axis / sinA;

                    // corrective torque (proportional to angle, scaled by stiffness and fragment mass)
                    Vector3 correctiveTorque = axisDir * (angle * k * f.mass);
                    // apply to fragment's torque accumulator
                    f.torque += correctiveTorque;
                }
            }

            // Any break happened this step? Add small lateral perturbation to excite wiggle
            bool brokeNow = false;
            for (int i = 0; i < constraints.Count; i++)
            {
                if (constraints[i].broken) brokeNow = true;
            }
            if (brokeNow && !anyBroken)
            {
                // If bottom joint snapped, add upward recoil to last link based on stored tension
                if (bottomJoint != null && bottomJoint.broken && generator.links.Count > 0)
                {
                    var lastLink = generator.links[generator.links.Count - 1];
                    Vector3 pA = lastLink.transform.position;
                    Vector3 pB = generator.bottomAnchor.transform.position;
                    Vector3 n = (pA - pB).normalized; // from bottom to link (upward along chain)
                    float recoilImpulse = Mathf.Clamp(bottomJoint.lastTension * timeStep, 0f, 10f * lastLink.mass);
                    lastLink.ApplyImpulseAtPoint(n * recoilImpulse, pA);
                }
                // Nudge links sideways slightly for visible wiggle
                foreach (var f in generator.links)
                {
                    Vector3 sideways = new Vector3(Random.Range(-0.15f, 0.15f), 0f, Random.Range(-0.15f, 0.15f));
                    f.ApplyImpulse(sideways * f.mass * 0.3f);
                }

                // Stop applying pull once the bottom is detached
                pullDisabledAfterBreak = true;

                // Restore other constraints to their normal break forces (if reinforced)
                if (preBreakReinforced)
                {
                    foreach (var c in constraints)
                    {
                        if (normalBreakForces.TryGetValue(c, out var fbf)) c.breakForce = fbf;
                    }
                    preBreakReinforced = false;
                }
            }
            anyBroken |= brokeNow;

            // Optional tension debug
            if (debugTension && bottomJoint != null && !pullDisabledAfterBreak)
            {
                Debug.Log($"Bottom tension: {bottomJoint.lastTension:F1} N  | breakForce: {bottomJoint.breakForce:F1} N");
            }

            // 4. Integrate positions & rotations
            foreach (var f in all)
            {
                if (f == top) continue; // keep fixed
                f.transform.position += f.linearVelocity * dt;
                Quaternion wq = new Quaternion(f.angularVelocity.x, f.angularVelocity.y, f.angularVelocity.z, 0f);
                Quaternion dq = ScaleQ(MulQ(wq, f.orientation), 0.5f * dt);
                f.orientation = NormalizeQ(AddQ(f.orientation, dq));
                f.transform.rotation = f.orientation;
            }

            // 5. Ground contact for links and bottom anchor
            HandleGround(dt);
        }

        void HandleGround(float dt)
        {
            foreach (var f in all)
            {
                if (f == generator.topAnchor) continue; // fixed
                float halfY = f.size.y * 0.5f;
                float pen = groundY + halfY - f.transform.position.y;
                if (pen > 0f)
                {
                    f.transform.position += Vector3.up * pen;
                    Vector3 contact = f.transform.position + new Vector3(0, -halfY, 0);
                    Vector3 v = f.GetVelocityAtPoint(contact);
                    Vector3 n = Vector3.up; float vn = Vector3.Dot(v, n);
                    if (vn < 0f)
                    {
                        Vector3 r = contact - f.transform.position;
                        Vector3 rXn = Vector3.Cross(r, n);
                        Vector3 ang = Matrix4x4Math.Multiply3x3(f.inertiaTensorWorldInverse, rXn);
                        float invMeff = 1f / f.mass + Vector3.Dot(Vector3.Cross(ang, r), n);
                        float meff = 1f / Mathf.Max(invMeff, 1e-5f);
                        float jn = -(1f + groundRestitution) * vn * meff;
                        f.ApplyImpulseAtPoint(n * jn, contact);

                        // Friction
                        Vector3 vt = v - vn * n; float vtLen = vt.magnitude;
                        if (vtLen > 1e-3f)
                        {
                            Vector3 t = vt / vtLen; float maxJt = groundFriction * Mathf.Abs(jn);
                            float jt = Mathf.Min(vtLen * meff, maxJt);
                            f.ApplyImpulseAtPoint(-t * jt, contact);
                        }
                    }

                    // rolling resistance
                    f.angularVelocity *= Mathf.Max(0f, 1f - rollingResistance * 4f * dt);
                }
            }
        }

        // quaternion helpers
        static Quaternion MulQ(Quaternion a, Quaternion b)
        {
            return new Quaternion(
                a.w * b.x + a.x * b.w + a.y * b.z - a.z * b.y,
                a.w * b.y - a.x * b.z + a.y * b.w + a.z * b.x,
                a.w * b.z + a.x * b.y - a.y * b.x + a.z * b.w,
                a.w * b.w - a.x * b.x - a.y * b.y - a.z * b.z
            );
        }
        static Quaternion AddQ(Quaternion a, Quaternion b) => new Quaternion(a.x + b.x, a.y + b.y, a.z + b.z, a.w + b.w);
        static Quaternion ScaleQ(Quaternion q, float s) => new Quaternion(q.x * s, q.y * s, q.z * s, q.w * s);
        static Quaternion NormalizeQ(Quaternion q)
        {
            float m = Mathf.Sqrt(q.x * q.x + q.y * q.y + q.z * q.z + q.w * q.w);
            if (m < 1e-6f) return Quaternion.identity;
            float im = 1f / m; return new Quaternion(q.x * im, q.y * im, q.z * im, q.w * im);
        }

        private void OnDrawGizmos()
        {
            if (generator == null) return;
            Gizmos.color = Color.yellow;
            foreach (var c in constraints)
            {
                if (c == null || c.broken) continue;
                Vector3 pA = c.A.transform.TransformPoint(c.localA);
                Vector3 pB = c.B.transform.TransformPoint(c.localB);
                Gizmos.DrawLine(pA, pB);
            }
            // Ground
            Gizmos.color = new Color(0.5f,0.5f,0.5f,0.5f);
            Gizmos.DrawWireCube(new Vector3(0, groundY, 0), new Vector3(20, 0.02f, 20));
        }
    }
}
