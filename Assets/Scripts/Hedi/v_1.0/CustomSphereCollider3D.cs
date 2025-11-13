using UnityEngine;

[DisallowMultipleComponent]
public class CustomSphereCollider3D : CustomCollider3D
{
    public float radius = 0.5f;

    public override Vector3 Center => transform.position;

    public override bool Intersects(CustomCollider3D other)
    {
        // Use AABB approximation for simplicity (fast, conservative for our manager)
        GetAABB(out Vector3 aCenter, out Vector3 aSize);
        other.GetAABB(out Vector3 bCenter, out Vector3 bSize);

        return (Mathf.Abs(aCenter.x - bCenter.x) * 2 < (aSize.x + bSize.x)) &&
               (Mathf.Abs(aCenter.y - bCenter.y) * 2 < (aSize.y + bSize.y)) &&
               (Mathf.Abs(aCenter.z - bCenter.z) * 2 < (aSize.z + bSize.z));
    }

    public override void GetAABB(out Vector3 center, out Vector3 size)
    {
        center = transform.position;
        float d = radius * 2f;
        size = new Vector3(d, d, d);
    }

    private void OnDrawGizmosSelected()
    {
        Gizmos.color = Color.cyan;
        Gizmos.DrawWireSphere(Center, radius);
    }
}
