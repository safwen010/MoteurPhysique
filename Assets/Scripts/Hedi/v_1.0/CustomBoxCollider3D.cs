using UnityEngine;
using System;

[DisallowMultipleComponent]
public class CustomBoxCollider3D : CustomCollider3D
{
    public Vector3 size = Vector3.one;
    public override Vector3 Center => transform.position;

    public override bool Intersects(CustomCollider3D other)
    {
        // Approximate using both AABBs
        GetAABB(out Vector3 aCenter, out Vector3 aSize);
        other.GetAABB(out Vector3 bCenter, out Vector3 bSize);

        return (Mathf.Abs(aCenter.x - bCenter.x) * 2 < (aSize.x + bSize.x)) &&
               (Mathf.Abs(aCenter.y - bCenter.y) * 2 < (aSize.y + bSize.y)) &&
               (Mathf.Abs(aCenter.z - bCenter.z) * 2 < (aSize.z + bSize.z));
    }

    public override void GetAABB(out Vector3 center, out Vector3 sizeOut)
    {
        center = transform.position;
        sizeOut = size;
    }

    private void OnDrawGizmosSelected()
    {
        Gizmos.color = Color.yellow;
        Gizmos.DrawWireCube(Center, size);
    }
}
