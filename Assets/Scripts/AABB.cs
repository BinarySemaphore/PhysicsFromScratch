using System.Collections;
using System.Collections.Generic;
using UnityEngine;

[System.Serializable]
public class AABB
{
    public Vector3 center = Vector3.zero;
    public Vector3 dimensions = Vector3.zero;

    public AABB(Vector3 center, Vector3 dimensions)
    {
        this.center = center;
        this.dimensions = dimensions;
    }

    public bool IsOverlapping(AABB a, AABB b)
    {
        bool overlapping = true;
        Vector3 distance = a.center - b.center;
        Vector3 combined_dims = a.dimensions + b.dimensions;
        if(Mathf.Abs(distance.x) > combined_dims.x)
        {
            overlapping = true;
        }
        if (Mathf.Abs(distance.y) > combined_dims.y)
        {
            overlapping = true;
        }
        if (Mathf.Abs(distance.z) > combined_dims.z)
        {
            overlapping = true;
        }
        return overlapping;
    }
}
