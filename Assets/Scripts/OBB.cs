using System.Collections.Generic;
using UnityEngine;

public class OBB
{
    private Vector3 center;
    private Vector3 dimensions;
    private Quaternion rotation;
    private Vector3[] vertices;

    /// <summary>
    /// Creates Oriented Bounding Box for precise collision detection and information.
    /// </summary>
    /// <param name="position"></param>
    /// <param name="rotation"></param>
    /// <param name="dimensions"></param>
    public OBB(Vector3 position, Quaternion rotation, Vector3 dimensions)
    {
        this.center = position;
        this.rotation = rotation;
        this.dimensions = dimensions;
        this.vertices = this.GetVertices(position, rotation);
    }

    /// <summary>
    /// Calculate the corner verticies of the <see cref="OBB"/>.
    /// </summary>
    /// <param name="position"></param>
    /// <param name="rotation"></param>
    /// <returns><see cref="Vector3"/> array of the vertices.</returns>
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

            // Update smallest intersecting distances and index (want to use near zero for comparison otherwise extremly small numbers will mess up normal from cross axis tests).
            if (distance_results[i] <= -0.000001f && distance_results[i] > smallest_intersecting_distance)
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
        Vector3 new_normal;
        position = OBB.GetPositionOfOverlap(A, B, out new_normal);
        //if (new_normal != Vector3.zero) normal = new_normal;
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

    /// <summary>
    /// Find average position of overlapping <paramref name="A"/> and <paramref name="B"/> using each edge and face.
    /// </summary>
    /// <remarks>Should only use if overlapping state of <paramref name="A"/> and <paramref name="B"/> are already known to be true.</remarks>
    /// <param name="A"></param>
    /// <param name="B"></param>
    /// <returns><see cref="Vector3"/> world position.</returns>
    private static Vector3 GetPositionOfOverlap(OBB A, OBB B, out Vector3 normal)
    {
        int count = 0;
        Vector3 location = Vector3.zero;
        Vector3 normals = Vector3.zero;
        normal = Vector3.zero;

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
                new int[] { 6, 4 }
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
                        normals += found_normal;
                    }
                }
            }
        }

        // If all checks missed A and B are perfectly overlapping
        if (count == 0) return 0.5f * (A.center + B.center);

        // Average the locations and normals
        location /= count;
        normal = (normals / count).normalized;

        return location;
    }

    /// <summary>
    /// Finds intersection along line defined by <paramref name="start"/> and <paramref name="end"/> with quad given with q-points.
    /// <para>Outs:<list type="bullet">
    /// <item><paramref name="location"/>: World location of the intersection if return true.</item>
    /// <item><paramref name="normal"/>: Normal of plane from intersection if return true.</item>
    /// </list></para>
    /// </summary>
    /// <remarks>Line and quad extension available to allow exceeding limits.</remarks>
    /// <param name="start"></param>
    /// <param name="end"></param>
    /// <param name="q1"></param>
    /// <param name="q2"></param>
    /// <param name="q3"></param>
    /// <param name="q4"></param>
    /// <param name="location"></param>
    /// <param name="normal"></param>
    /// <returns><see cref="System.Boolean"/> state of intersection.</returns>
    private static bool RayIntersectingQuad(Vector3 start, Vector3 end, Vector3 q1, Vector3 q2, Vector3 q3, Vector3 q4, out Vector3 location, out Vector3 normal)
    {
        location = Vector3.zero;

        float line_extension = 0.1f; // extends line limit before and after (zero is exact)
        float quad_extension = 0.1f; // extends quad range (zero is exact)

        float limit = (end - start).magnitude;
        Vector3 line_normal = (end - start).normalized;
        Vector3 plane_normal = Vector3.Cross(q2 - q1, q3 - q1).normalized;
        normal = plane_normal;

        /*
         * Line = a1, a2
         * Plane = q1, q2, q3
         *
         * points on the line:
         * n = normal(a2 - a1)
         * p = a1 + d * n | where d is distance from a1.
         *
         * points on the plane:
         * N = normal(crossProduct(q2 - q1, q3 - q1))
         * 0 = dotProduct(P - q1, N)
         *
         * Solve for d (distance to plane from a1):
         * P = p (a point on the line).
         * 0 = N.x * P.x - N.x * q1.x + N.y * P.y - N.y * q1.y + N.z * P.z - N.z * q1.z
         * N.x * q1.x + N.y * q1.y + N.z * q1.z = N.x * P.x + N.y * P.y + N.z * P.z
         * dotProduct(N, q1) = N.x * (a1.x + d * n.x) + N.y * (a1.y + d * n.y) + N.z * (a1.z + d * n.z)
         * dotProduct(N, q1) = N.x * a1.x + N.x * d * n.x + N.y * a1.y + N.y * d * n.y + N.z * a1.z + N.z * d * n.z
         * dotProduct(N, q1) - N.x * a1.x - N.y * a1.y - N.z * a1.z = N.x * d * n.x + N.y * d * n.y + N.z * d * n.z
         * dotProduct(N, q1) - dotProduct(N, a1) = d * (N.x * n.x + N.y * n.y + N.z * n.z)
         * d = (dotProduct(N, q1) - dotProduct(N, a1)) / dotProduct(N, n)
         */

        // Check if parallel
        float direction_compare = Vector3.Dot(line_normal, plane_normal);
        if (direction_compare == 0f) return false;

        // Get distance to plane from line start
        float distance = (Vector3.Dot(q1, plane_normal) - Vector3.Dot(start, plane_normal)) / direction_compare;

        // Check if distance is within limit (length of the line from start and end)
        if (distance < -line_extension || distance > limit + line_extension) return false;

        // Get the point on the plane along the line
        location = start + distance * line_normal;

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
        if (theta >= 2 * Mathf.PI - quad_extension && theta <= 2 * Mathf.PI + quad_extension) return true;

        return false;
    }
}
