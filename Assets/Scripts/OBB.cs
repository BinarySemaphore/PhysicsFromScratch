using System.Collections.Generic;
using UnityEngine;

public class OBB
{
    private Vector3 center;
    private Vector3 dimensions;
    private Quaternion rotation;
    private Vector3[] vertices;

    public OBB(Vector3 position, Quaternion rotation, Vector3 dimensions)
    {
        this.center = position;
        this.rotation = rotation;
        this.dimensions = dimensions;
        this.vertices = this.GetVertices(position, rotation);
    }

    private Vector3[] GetVertices(Vector3 position, Quaternion rotation)
    {
        Vector3 pos = 0.5f * this.dimensions;
        Vector3 neg = -0.5f * this.dimensions;

        Vector3[] vertices = new Vector3[8];
        vertices[0] = position + rotation * new Vector3(pos.x, pos.y, pos.z);
        vertices[1] = position + rotation * new Vector3(neg.x, pos.y, pos.z);
        vertices[2] = position + rotation * new Vector3(pos.x, neg.y, pos.z);
        vertices[3] = position + rotation * new Vector3(neg.x, neg.y, pos.z);
        vertices[4] = position + rotation * new Vector3(pos.x, pos.y, neg.z);
        vertices[5] = position + rotation * new Vector3(neg.x, pos.y, neg.z);
        vertices[6] = position + rotation * new Vector3(pos.x, neg.y, neg.z);
        vertices[7] = position + rotation * new Vector3(neg.x, neg.y, neg.z);

        return vertices;
    }

    public List<Vector3> GetVerticies()
    {
        return new List<Vector3> (this.vertices);
    }

    /// <summary>
    /// Check if oreiented bounding boxes <paramref name="A"/> overlap <paramref name="B"/> using SAT (separation axis theorem).
    /// <para>Outs:<list type="bullet">
    /// <item><paramref name="position"/>: Center of overlap.</item>
    /// <item><paramref name="normal"/>: Least overlapping axis.</item>
    /// <item><paramref name="depth"/>: Least overlapping amount.</item>
    /// </list></para>
    /// </summary>
    /// <param name="A"></param>
    /// <param name="B"></param>
    /// <param name="position"></param>
    /// <param name="normal"></param>
    /// <param name="depth"></param>
    /// <returns><see cref="System.Boolean"/> overlapping status.</returns>
    public static bool IsOverlapping(OBB A, OBB B, out Vector3 position, out Vector3 normal, out float depth)
    {
        depth = 0f;
        position = A.center;
        normal = Vector3.forward;

        int smallest_intersecting_distance_index = 0;
        float smallest_intersecting_distance = float.MinValue;
        int[] normals = new int[15];
        float[] distance_results = new float[15];
        Vector3[] axis_tests = new Vector3[15];
        axis_tests[0] = A.rotation * Vector3.forward;                 // A front
        axis_tests[1] = A.rotation * Vector3.up;                      // A top
        axis_tests[2] = A.rotation * Vector3.right;                   // A right
        axis_tests[3] = B.rotation * Vector3.forward;                 // B front
        axis_tests[4] = B.rotation * Vector3.up;                      // B top
        axis_tests[5] = B.rotation * Vector3.right;                   // B right
        axis_tests[6] = Vector3.Cross(axis_tests[0], axis_tests[3]);  // A front x B front
        axis_tests[7] = Vector3.Cross(axis_tests[0], axis_tests[4]);  // A front x B top
        axis_tests[8] = Vector3.Cross(axis_tests[0], axis_tests[5]);  // A front x B right
        axis_tests[9] = Vector3.Cross(axis_tests[1], axis_tests[3]);  // A top x B front
        axis_tests[10] = Vector3.Cross(axis_tests[1], axis_tests[4]); // A top x B top
        axis_tests[11] = Vector3.Cross(axis_tests[1], axis_tests[5]); // A top x B right
        axis_tests[12] = Vector3.Cross(axis_tests[2], axis_tests[3]); // A right x B front
        axis_tests[13] = Vector3.Cross(axis_tests[2], axis_tests[4]); // A right x B top
        axis_tests[14] = Vector3.Cross(axis_tests[2], axis_tests[5]); // A right x B right
        Vector3[] center_to_position = new Vector3[15];

        float center_on_axis;
        // Check for separation from: 3 of A's faces; 3 of B's faces; 9 planes to each combination of A and B faces
        for (int i = 0; i < 15; i++)
        {
            distance_results[i] = OBB.DistanceOverAxis(A, B, axis_tests[i], out normals[i], out center_on_axis);

            // Any separation axis with distance larger than zero percludes actual overlap
            if (distance_results[i] > 0f) return false;

            center_to_position[i] = center_on_axis * axis_tests[i];

            // Update smallest intersecting distances and index
            if (distance_results[i] != 0f && distance_results[i] > smallest_intersecting_distance)
            {
                smallest_intersecting_distance_index = i;
                smallest_intersecting_distance = distance_results[i];
            }
        }


        // Get normal from least overlapping axis
        normal = axis_tests[smallest_intersecting_distance_index].normalized;
        normal *= normals[smallest_intersecting_distance_index];

        // Get depth from least overlapping amount
        depth = -1f * smallest_intersecting_distance;

        // Get position
        position = center_to_position[0];
        for (int i = 1; i < 15; i++)
        {
            position += center_to_position[i];
        }
        position /= 4;
        position += B.center;
        
        return true;
    }

    /// <summary>
    /// Find edge to edge distance between <paramref name="A"/> and <paramref name="B"/> along <paramref name="axis"/>.
    /// <para>Outs:<list type="bullet">
    /// <item><paramref name="normal"/>: Direction to A along <paramref name="axis"/>.</item>
    /// <item><paramref name="center"/>: </item>
    /// </list></para>
    /// </summary>
    /// <param name="A"></param>
    /// <param name="B"></param>
    /// <param name="axis"></param>
    /// <param name="normal"></param>
    /// <param name="center"></param>
    /// <returns></returns>
    public static float DistanceOverAxis(OBB A, OBB B, Vector3 axis, out int normal, out float center)
    {
        normal = 1;
        center = 0.0f;

        // Find largest and smallest 1d lengths along axis from A and B vertices
        float min_A = float.MaxValue;
        float max_A = float.MinValue;
        float min_B = float.MaxValue;
        float max_B = float.MinValue;

        for (int i = 0; i < 8; i++)
        {
            float vert_A = Vector3.Dot(A.vertices[i], axis);
            float vert_B = Vector3.Dot(B.vertices[i], axis);
            if (vert_A < min_A) min_A = vert_A;
            if (vert_A > max_A) max_A = vert_A;
            if (vert_B < min_B) min_B = vert_B;
            if (vert_B > max_B) max_B = vert_B;
        }

        // Example: |---------------coverage-------------|
        //          |---fill_A----|          |--fill_B---|
        //        min_A---------max_A------min_B-------max_B
        float coverage = Mathf.Max(max_A, max_B) - Mathf.Min(min_A, min_B);
        float fill_A = max_A - min_A;
        float fill_B = max_B - min_B;
        float combined_fill = fill_A + fill_B;

        // Get center
        float center_A = (max_A - min_A) * 0.5f + min_A;
        float center_B = (max_B - min_B) * 0.5f + min_B;
        center = 0.5f * (center_A - center_B);

        // Get normal
        if (center_A < center_B)
        {
            normal = -1;
        }
        else
        {
            normal = 1;
        }

        return coverage - combined_fill;
    }
}
