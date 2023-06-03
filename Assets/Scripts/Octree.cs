using System.Collections;
using System.Collections.Generic;
using UnityEngine;

/// <summary>
/// Wraps <see cref="AABB"/> for subdivision awareness.
/// </summary>
public class OctreeItem : AABB
{
    public List<List<Octree>> subdivisions;

    public OctreeItem(GameObject entity) : base(entity, AABB.Containment.largest)
    {
        subdivisions = new List<List<Octree>>();
    }

    public void AddSubdivision(int depth, Octree subdivision)
    {
        if (this.subdivisions.Count <= depth) this.subdivisions.Add(new List<Octree>());
        this.subdivisions[depth].Add(subdivision);
    }

    public void ResetSubdivisions()
    {
        this.subdivisions = new List<List<Octree>>();
    }
}

/// <summary>
/// Octree AABB spacial structure.
/// </summary>
public class Octree : AABB
{
    public int index;
    public List<Octree> children;
    public List<OctreeItem> items;

    /// <summary>
    /// Manually creates empty Octree node
    /// </summary>
    /// <param name="center"></param>
    /// <param name="dimensions"></param>
    public Octree(Vector3 center, Vector3 dimensions) : base(center, dimensions)
    {
        this.index = 0;
        this.children = new List<Octree>();
        this.items = new List<OctreeItem>();
    }

    /// <summary>
    /// Creates Octree root containing all <paramref name="items"/>.
    /// </summary>
    /// <param name="items"></param>
    public Octree(List<OctreeItem> items) : base(Vector3.zero, Vector3.zero)
    {
        Vector3 max = Vector3.zero;
        Vector3 min = Vector3.zero;

        foreach (OctreeItem item in items)
        {
            Vector3 max_corner = item.center + item.dimensions;
            Vector3 min_corner = item.center - item.dimensions;

            if (max == Vector3.zero)
            {
                max = max_corner;
            }
            else
            {
                if (max_corner.x > max.x) max.x = max_corner.x;
                if (max_corner.y > max.y) max.y = max_corner.y;
                if (max_corner.z > max.z) max.z = max_corner.z;
            }

            if (min == Vector3.zero)
            {
                min = min_corner;
            }
            else
            {
                if (min_corner.x < min.x) min.x = min_corner.x;
                if (min_corner.y < min.y) min.y = min_corner.y;
                if (min_corner.z < min.z) min.z = min_corner.z;
            }
        }

        this.center = 0.5f * (max + min);
        this.dimensions = 0.5f * (max - min);
        this.index = 0;
        this.children = new List<Octree>();
        this.items = items;
    }

    /// <summary>
    /// Generates eight child <see cref="Octree"/> nodes equally dividing the <paramref name="parent"/>.
    /// </summary>
    /// <remarks><see cref="items"/> are empty and must be filled, <see cref="Subdivide(Octree, int)"/>.</remarks>
    /// <param name="parent"></param>
    /// <returns>List of eight new <see cref="Octree"/> nodes.</returns>
    public static List<Octree> OctreesFromParent(Octree parent)
    {
        List<Octree> new_octrees = new List<Octree>();
        Vector3 half_dimensions = 0.5f * parent.dimensions;

        for (int i = 0; i < 8; i++)
        {
            Vector3 scale = Vector3.one;
            if (i % 2 == 1) scale.x = -1.0f;
            if (Mathf.Floor((i % 4) / 2) == 1) scale.y = -1.0f;
            if (Mathf.Floor(i / 4) == 1) scale.z = -1.0f;

            Vector3 offset = Vector3.Scale(half_dimensions, scale);
            Octree new_octree = new Octree(parent.center + offset, half_dimensions);
            new_octree.index = i;
            parent.children.Add(new_octree);
            new_octrees.Add(new_octree);
        }

        return new_octrees;
    }

    /// <summary>
    /// Recursively subdivide <paramref name="current"/> until constraints are met.
    /// <para>Constraints:
    /// <list type="bullet">
    ///     <item>Five or fewer <see cref="items"/></item>
    ///     <item>Any axis of <see cref="AABB.dimensions"/> less than 5.0f</item>
    /// </list></para>
    /// </summary>
    /// <remarks>Refuses to subdivide if <see cref="items"/> do not reduce.</remarks>
    /// <param name="current"></param>
    /// <param name="depth"></param>
    public static void Subdivide(Octree current, int depth = 0)
    {
        // Update items' subdivision awareness
        foreach (OctreeItem item in current.items)
        {
            item.AddSubdivision(depth, current);
        }

        // Check subdivision min constraints
        if (current.items.Count <= 3) return;
        if (current.dimensions.x < 5.0f || current.dimensions.y < 5.0f || current.dimensions.z < 5.0f) return;

        // Check if subdividing required
        foreach (Octree subdivision in Octree.OctreesFromParent(current))
        {
            List<OctreeItem> found_items = subdivision.items;
            foreach (OctreeItem item in current.items)
            {
                if (AABB.IsOverlapping(item, subdivision))
                {
                    found_items.Add(item);
                }
            }

            // If items reduced by subdivision, then vaild subdivision and continue
            if (found_items.Count < current.items.Count)
            {
                Octree.Subdivide(subdivision, depth + 1);
            }
        }
    }
}
