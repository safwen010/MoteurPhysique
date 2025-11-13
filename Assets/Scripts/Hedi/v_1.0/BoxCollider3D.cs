using UnityEngine;
using System;

public class BoxCollider3D
{
    public Vec3 center;
    public Vec3 size;

    public BoxCollider3D(Vec3 center, Vec3 size)
    {
        this.center = center;
        this.size = size;
    }

    public bool Intersects(BoxCollider3D other)
    {
        return (Math.Abs(center.x - other.center.x) * 2 < (size.x + other.size.x)) &&
               (Math.Abs(center.y - other.center.y) * 2 < (size.y + other.size.y)) &&
               (Math.Abs(center.z - other.center.z) * 2 < (size.z + other.size.z));
    }
}
