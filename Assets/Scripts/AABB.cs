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
    /// Manually defined Axis Aligned Bounding Box.
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

    /// <summary>
    /// Check if <paramref name="A"/> and <paramref name="B"/> are overlapping.
    /// </summary>
    /// <param name="A"></param>
    /// <param name="B"></param>
    /// <returns><see cref="System.Boolean"/> overlapping status.</returns>
    public static bool IsOverlapping(AABB A, AABB B)
    {
        bool overlapping = true;
        Vector3 distance = A.center - B.center;
        Vector3 combined_dims = A.dimensions + B.dimensions;
        if(Mathf.Abs(distance.x) > combined_dims.x)
        {
            overlapping = false;
        }
        if (Mathf.Abs(distance.y) > combined_dims.y)
        {
            overlapping = false;
        }
        if (Mathf.Abs(distance.z) > combined_dims.z)
        {
            overlapping = false;
        }
        return overlapping;
    }

    /// <summary>
    /// Return a face normal of overlapping <see cref="AABB"/>s.
    /// </summary>
    /// <param name="A"></param>
    /// <param name="B"></param>
    /// <returns>face axis normal <see cref="Vector3"/>.</returns>
    public static Vector3 NormalOfOverlapping(AABB A, AABB B)
    {
        Vector3 normal = Vector3.zero;
        Vector3 distance = A.center - B.center;
        Vector3 combined_dims = A.dimensions + B.dimensions;
        float x_diff = combined_dims.x - Mathf.Abs(distance.x);
        float y_diff = combined_dims.y - Mathf.Abs(distance.y);
        float z_diff = combined_dims.z - Mathf.Abs(distance.z);
        float min_diff = Mathf.Min(x_diff, y_diff, z_diff);

        if (x_diff == min_diff) normal.x = 1.0f;
        else if (y_diff == min_diff) normal.y = 1.0f;
        else if (z_diff == min_diff) normal.z = 1.0f;
        
        return normal;
    }

    /// <summary>
    /// Get corner points for drawing.
    /// </summary>
    /// <returns>List of eight corner points</returns>
    public List<Vector3> GetDrawPoints()
    {
        List<Vector3> points = new List<Vector3>();
        for (int i=0; i < 8; i++)
        {
            /*
             * Dimension Expression Order:
             * 0:  1,  1,  1
             * 1: -1,  1,  1
             * 2:  1, -1,  1
             * 3: -1, -1,  1
             * 4:  1,  1, -1
             * 5: -1,  1, -1
             * 6:  1, -1, -1
             * 7: -1, -1, -1
             */
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
