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
        position = OBB.GetPositionOfOverlap(A, B);
        /*
        position = center_to_position[0];
        for (int i = 1; i < 15; i++)
        {
            position += center_to_position[i];
        }
        position /= 4;
        position += B.center;
        */
        
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
    private static float DistanceOverAxis(OBB A, OBB B, Vector3 axis, out int normal, out float center)
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

    private static Vector3 GetPositionOfOverlap(OBB A, OBB B)
    {
        int count = 0;
        Vector3 location = Vector3.zero;
        Vector3 normal = Vector3.zero;

        // Check edges with front: 0-1, 1-3, 3-2, 2-0 | back: 4-5, 5-7, 7-6, 6-0 | cross: 0-4, 1-5, 2-6, 3-7
        int[][][] edge_pair_indecies = new int[][][]
        {
            new int[][]
            {
                new int[] { 0, 1 },
                new int[] { 1, 3 },
                new int[] { 3, 2 },
                new int[] { 2, 0 }
            },
            new int[][]
            {
                new int[] { 4, 5 },
                new int[] { 5, 7 },
                new int[] { 7, 6 },
                new int[] { 6, 0 }
            },
            new int[][]
            {
                new int[] { 0, 4 },
                new int[] { 1, 5 },
                new int[] { 2, 6 },
                new int[] { 3, 7 }
            }
        };
        int[][] face_indecies = new int[][]
        {
            new int[] { 0, 1, 3, 2 }, // front
            new int[] { 4, 5, 7, 6 }, // back
            new int[] { 0, 1, 5, 4 }, // top
            new int[] { 0, 4, 6, 2 }, // right
            new int[] { 6, 7, 3, 2 }, // bottom
            new int[] { 5, 1, 3, 7 }  // left
        };

        // Check A faces against B edges
        Vector3 found_location;
        Vector3 found_normal;
        Vector3 face_p1;
        Vector3 face_p2;
        Vector3 face_p3;
        Vector3 face_p4;
        Vector3 edge_p1;
        Vector3 edge_p2;
        foreach (int[] face in face_indecies)
        {
            face_p1 = B.vertices[face[0]];
            face_p2 = B.vertices[face[1]];
            face_p3 = B.vertices[face[2]];
            face_p4 = B.vertices[face[3]];

            foreach (int[][] face_edges in edge_pair_indecies)
            {
                foreach (int[] edge_pair in face_edges)
                {
                    edge_p1 = A.vertices[edge_pair[0]];
                    edge_p2 = A.vertices[edge_pair[1]];

                    if (OBB.RayIntersectingQuad(edge_p1, edge_p2, face_p1, face_p2, face_p3, face_p4, out found_location, out found_normal))
                    {
                        count += 1;
                        location += found_location;
                        normal += found_normal;
                    }
                }
            }
        }

        // Check B faces against A edges
        foreach (int[] face in face_indecies)
        {
            face_p1 = A.vertices[face[0]];
            face_p2 = A.vertices[face[1]];
            face_p3 = A.vertices[face[2]];
            face_p4 = A.vertices[face[3]];

            foreach (int[][] face_edges in edge_pair_indecies)
            {
                foreach (int[] edge_pair in face_edges)
                {
                    edge_p1 = B.vertices[edge_pair[0]];
                    edge_p2 = B.vertices[edge_pair[1]];

                    if (OBB.RayIntersectingQuad(edge_p1, edge_p2, face_p1, face_p2, face_p3, face_p4, out found_location, out found_normal))
                    {
                        count += 1;
                        location += found_location;
                        normal += found_normal;
                    }
                }
            }
        }

        // Average the locations and normals
        location /= count;
        normal /= count;
        /*
        int[] face = face_indecies[3];
        int[][] edge = edge_pair_indecies[0];
        int[] edge_pair = edge[0];
        if (RayIntersectingQuad(
            A.vertices[edge_pair[0]], A.vertices[edge_pair[1]],
            B.vertices[face[0]], B.vertices[face[1]], B.vertices[face[2]], B.vertices[face[3]],
            out location, out normal))
        {
            return location;
        }
        */

        return location;
    }

    private static bool RayIntersectingQuad(Vector3 start, Vector3 end, Vector3 q1, Vector3 q2, Vector3 q3, Vector3 q4, out Vector3 location, out Vector3 normal)
    {
        location = Vector3.zero;

        float limit = (end - start).magnitude;
        Vector3 ray_direction = end - start;
        Vector3 plane_normal = Vector3.Cross(q2 - q1, q3 - q1);
        normal = plane_normal;

        // Get start_end vector projected onto plane described by q1, q2, q3
        float plane_equation = q1.x * plane_normal.x + q1.y * plane_normal.y + q1.z * plane_normal.z;
        float plane_solve_ray_start = plane_equation - start.x * plane_normal.x - start.y * plane_normal.y - start.z * plane_normal.z;
        float plane_solve_ray_direction = ray_direction.x * plane_normal.x + ray_direction.y * plane_normal.y + ray_direction.z * plane_normal.z;
        float solve_ray = plane_solve_ray_start / plane_solve_ray_direction;

        // Check if there is a solution to projection within limit
        if (solve_ray < 0.0f || solve_ray > limit) return false;

        // Get the actual point on the plane
        ray_direction = solve_ray * ray_direction.normalized;
        location = start + ray_direction;

        // Check if projection is within quad (sum of anglea from each location to point on edge)
        float theta = 0.0f;
        Vector3 check_1, check_2;

        check_1 = q1 - location;
        check_2 = q2 - location;
        theta += Mathf.Acos(Vector3.Dot(check_1.normalized, check_2.normalized));

        check_1 = q2 - location;
        check_2 = q3 - location;
        theta += Mathf.Acos(Vector3.Dot(check_1.normalized, check_2.normalized));

        check_1 = q3 - location;
        check_2 = q4 - location;
        theta += Mathf.Acos(Vector3.Dot(check_1.normalized, check_2.normalized));

        check_1 = q4 - location;
        check_2 = q1 - location;
        theta += Mathf.Acos(Vector3.Dot(check_1.normalized, check_2.normalized));

        // Check if sum of all angles (theta) is close to 360 (2pi)
        if (theta > 2 * Mathf.PI - 0.001f && theta < 2 * Mathf.PI + 0.001f) return true;

        return false;
    }
}
