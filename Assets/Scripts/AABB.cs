using System.Collections;
using System.Collections.Generic;
using UnityEngine;

/// <summary>
/// Axis Aligned Bounding Box
/// </summary>
[System.Serializable]
public class AABB
{
    public enum Containment {
        smallest = 0,
        largest = 1
    }
    public Vector3 center = Vector3.zero;
    public Vector3 dimensions = Vector3.zero;

    /// <summary>
    /// Manually defined AABB
    /// </summary>
    /// <param name="center"></param>
    /// <param name="dimensions"></param>
    public AABB(Vector3 center, Vector3 dimensions)
    {
        this.center = center;
        this.dimensions = dimensions;
    }

    /// <summary>
    /// AABB for <paramref name="entity"/>.
    /// </summary>
    /// <param name="entity"></param>
    /// <param name="type">Largest easiest and smallest costly.</param>
    public AABB(GameObject entity, Containment type)
    {
        this.center = entity.transform.position;

        if (type == Containment.smallest)
        {
            Vector3 dims_pos = entity.transform.localScale * 0.495f;
            Vector3 dims_neg = entity.transform.localScale * -0.495f;

            Vector3 corner_aa = entity.transform.rotation * new Vector3(dims_neg.x, dims_neg.y, dims_pos.z);
            Vector3 corner_ab = entity.transform.rotation * new Vector3(dims_pos.x, dims_neg.y, dims_pos.z);
            Vector3 corner_ac = entity.transform.rotation * new Vector3(dims_neg.x, dims_pos.y, dims_pos.z);
            Vector3 corner_ad = entity.transform.rotation * new Vector3(dims_pos.x, dims_pos.y, dims_pos.z);

            this.dimensions = new Vector3(
                Mathf.Max(Mathf.Abs(corner_aa.x), Mathf.Abs(corner_ab.x), Mathf.Abs(corner_ac.x), Mathf.Abs(corner_ad.x)),
                Mathf.Max(Mathf.Abs(corner_aa.y), Mathf.Abs(corner_ab.y), Mathf.Abs(corner_ac.y), Mathf.Abs(corner_ad.y)),
                Mathf.Max(Mathf.Abs(corner_aa.z), Mathf.Abs(corner_ab.z), Mathf.Abs(corner_ac.z), Mathf.Abs(corner_ad.z))
            );
        }
        else if (type == Containment.largest)
        {
            float largest_dim = entity.transform.localScale.magnitude * 0.5f;
            this.dimensions = new Vector3(largest_dim, largest_dim, largest_dim);
        }
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

    public List<Vector3> GetDrawPoints()
    {
        List<Vector3> points = new List<Vector3>();
        for (int i=0; i < 8; i++)
        {
            Vector3 modifier = Vector3.one;
            if (i % 2 == 1) modifier.x = -1;
            if (Mathf.Floor((i % 4) / 2) == 1) modifier.y = -1;
            if (Mathf.Floor(i / 4) == 1) modifier.z = -1;

            Vector3 new_point = Vector3.Scale(this.dimensions, modifier);
            points.Add(new_point + center);
        }
        return points;
    }
}
