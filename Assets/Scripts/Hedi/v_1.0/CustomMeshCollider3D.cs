using UnityEngine;

[DisallowMultipleComponent]
[RequireComponent(typeof(MeshFilter))]
public class CustomMeshCollider3D : CustomCollider3D
{
    // Optional MeshFilter on the same GameObject; we use its renderer.bounds for a conservative AABB
    // Recompute bounds from the actual mesh vertices so the collider follows mesh deformation/animation
    private Mesh bakedMesh;

    public override Vector3 Center
    {
        get
        {
            GetAABB(out Vector3 c, out Vector3 s);
            return c;
        }
    }

    public override bool Intersects(CustomCollider3D other)
    {
        GetAABB(out Vector3 aCenter, out Vector3 aSize);
        other.GetAABB(out Vector3 bCenter, out Vector3 bSize);

        return (Mathf.Abs(aCenter.x - bCenter.x) * 2 < (aSize.x + bSize.x)) &&
               (Mathf.Abs(aCenter.y - bCenter.y) * 2 < (aSize.y + bSize.y)) &&
               (Mathf.Abs(aCenter.z - bCenter.z) * 2 < (aSize.z + bSize.z));
    }

    public override void GetAABB(out Vector3 center, out Vector3 size)
    {
        // Try SkinnedMeshRenderer first (animated meshes)
        var smr = GetComponent<SkinnedMeshRenderer>();
        Vector3[] verts = null;

        if (smr != null)
        {
            if (bakedMesh == null) bakedMesh = new Mesh();
            smr.BakeMesh(bakedMesh);
            verts = bakedMesh.vertices;
        }
        else
        {
            var mf = GetComponent<MeshFilter>();
            var mesh = mf != null ? mf.sharedMesh : null;
            if (mesh != null)
                verts = mesh.vertices;
        }

        if (verts == null || verts.Length == 0)
        {
            center = transform.position;
            size = Vector3.zero;
            return;
        }

        // Transform local vertices to world space and compute min/max
        Matrix4x4 localToWorld = transform.localToWorldMatrix;
        Vector3 v0 = localToWorld.MultiplyPoint3x4(verts[0]);
        Vector3 min = v0;
        Vector3 max = v0;
        for (int i = 1; i < verts.Length; i++)
        {
            Vector3 w = localToWorld.MultiplyPoint3x4(verts[i]);
            min = Vector3.Min(min, w);
            max = Vector3.Max(max, w);
        }

        center = (min + max) * 0.5f;
        size = max - min;
    }

    private void OnDrawGizmosSelected()
    {
        Gizmos.color = Color.magenta;
        GetAABB(out Vector3 c, out Vector3 s);
        // Draw wire cube of the computed AABB
        Gizmos.DrawWireCube(c, s);
    }

    void OnDestroy()
    {
        if (bakedMesh != null)
        {
            Destroy(bakedMesh);
            bakedMesh = null;
        }
    }
}
