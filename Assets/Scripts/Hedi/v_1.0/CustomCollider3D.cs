using UnityEngine;

// Base class for custom colliders so the physics manager can handle different shapes
public abstract class CustomCollider3D : MonoBehaviour
{
    // World-space center of the collider
    public abstract Vector3 Center { get; }

    // Return true if this collider intersects the other collider (approx via AABB)
    public abstract bool Intersects(CustomCollider3D other);

    // Provide an AABB for coarse collision tests (center and size)
    public abstract void GetAABB(out Vector3 center, out Vector3 size);
}
