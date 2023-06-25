using System.Collections.Generic;
using UnityEngine;

public class Collision
{
    private static readonly float disp_offset = 1.01f;
    private static readonly float react_offset = 1.02f;

    public Body A;
    public Body B;
    public float depth;
    public Vector3 location;
    public Vector3 normal;

    /// <summary>
    /// Collision information.
    /// </summary>
    /// <param name="A"></param>
    /// <param name="B"></param>
    /// <param name="location"></param>
    /// <param name="normal"></param>
    public Collision(Body A, Body B, Vector3 location, Vector3 normal, float depth)
    {
        this.A = A;
        this.B = B;
        this.depth = depth;
        this.location = location;
        this.normal = normal;
    }

    /// <summary>
    /// Get <see cref="Collision"/>s between <paramref name="A"/> and ground tagged GameObjects.
    /// </summary>
    /// <param name="A"></param>
    /// <param name="delta_time"></param>
    /// <returns><see cref="List{Collision}"/> any collisions</returns>
    public static List<Collision> GetCollisionsForGround(Body A, float delta_time)
    {
        List<Collision> collisions = new List<Collision>();
        float velocity_down = -1 * delta_time * A.velocity.y;
        float dims_y = 0.5f * Mathf.Abs((A.transform.rotation * A.transform.localScale).y);
        //float distance = 2 * (dims_y + velocity_down);
        //Vector3 origin = A.transform.position;
        //origin.y += dims_y + velocity_down;
        OBB precise_box = A.GetPreciseCollider();

        int count = 0;
        float depth = 0f;
        Vector3 position = Vector3.zero;
        Vector3 normal = Vector3.zero;

        // 
        foreach (Vector3 vertex in precise_box.GetVerticies())
        {
            float distance_to_origin = A.transform.position.y - vertex.y;
            // Skip if above A's center point
            if (distance_to_origin < 0f) continue;

            Vector3 origin = vertex;
            origin.y += 1.0f;

            float distance_to_check = 1.0f;

            if (Physics.Raycast(origin, Vector3.down, out RaycastHit hit, distance_to_check))
            {
                if (hit.collider.CompareTag(A.sim.grounds_tag))
                {
                    count += 1;
                    depth += hit.point.y - vertex.y;
                    position += hit.point;
                    normal += hit.normal;
                }
            }
        }

        if (count == 0) return collisions;

        depth /= count;
        position /= count;
        normal /= count;
        Debug.Log($"Depth: {depth}");
        collisions.Add(new Collision(
            A, null,
            position,
            normal,
            depth
        ));

        return collisions;
    }

    /// <summary>
    /// Get <see cref="Collision"/>s between <paramref name="A"/> and <paramref name="neighbors"/>. 
    /// </summary>
    /// <param name="A"></param>
    /// <param name="neighbors"></param>
    /// <returns><see cref="List{Collision}"/> any collisions</returns>
    public static List<Collision> GetCollisionsForDynamicNeighbors(Body A, List<Body> neighbors)
    {
        List<Collision> collisions = new List<Collision>();
        foreach (Body neighbor in neighbors)
        {
            if (AABB.IsOverlapping(A.bounding_box, neighbor.bounding_box))
            {
                OBB precise_box_A = A.GetPreciseCollider();
                OBB precise_box_B = neighbor.GetPreciseCollider();
                if (OBB.IsOverlapping(precise_box_A, precise_box_B, out Vector3 location, out Vector3 normal, out float depth))
                {
                    collisions.Add(new Collision(
                        A, neighbor,
                        location,
                        normal,
                        depth
                    ));
                }
            }
        }
        return collisions;
    }

    /// <summary>
    /// Resolve collision by applying <see cref="Body.AccumulationType"/> to <see cref="Body"/>.
    /// </summary>
    /// <param name="A"></param>
    /// <param name="position"></param>
    /// <param name="normal"></param>
    /// <param name="delta_time"></param>
    public static void RespondDynamicAGroundB(Body A, Vector3 position, Vector3 normal, float depth, float delta_time)
    {
        float friction = A.friction;
        float elasticity = A.elasticity;

        Vector3 position_to_A = A.transform.position - position;

        Vector3 velocity_A_from_rotation = Vector3.Cross(position_to_A, A.r_velocity);
        Vector3 velocity_A_at_position = A.velocity + velocity_A_from_rotation;

        float acting_velocity_A = Vector3.Dot(velocity_A_at_position, normal);
        float relative_velocity = acting_velocity_A;

        if (relative_velocity < 0f)
        {
            float inv_distance_to_A = 1f / position_to_A.magnitude;
            Vector3 direction_to_A = position_to_A * inv_distance_to_A;

            // Get distribution percents along the normal and direction to center of mass
            float linear_percent_A = Mathf.Abs(Vector3.Dot(direction_to_A, normal));
            float angular_percent_A = 1f - linear_percent_A;

            float reactive_velocity_A = 0f;

            if (relative_velocity <= -2f)
            {
                // Get relative moment of inertia at position along normal
                float moment_A = Vector3.Dot(A.GetRelativeMoment(), normal);

                // Get total momentum and mass using distributions
                float total_inertial_mass = linear_percent_A * A.mass +
                                            angular_percent_A * moment_A;

                //Get velocity change due to impact reaction
                // -e(v1 - v2) / total_mass  <- B is static and has infinite mass which cancel m2 and total_momentum aproaches zero from total_inertial_mass
                reactive_velocity_A = -elasticity * relative_velocity;
                reactive_velocity_A /= total_inertial_mass;
            }

            //Get velocity change due to friction
            Vector3 planar_friction_A = -1f * friction * (velocity_A_at_position - acting_velocity_A * normal);

            //Get total delta velocity required
            Vector3 delta_velocity_A = planar_friction_A + (reactive_velocity_A - Collision.react_offset * acting_velocity_A) * normal;

            Vector3 delta_linear_velocity_A = linear_percent_A * delta_velocity_A;

            Vector3 delta_angular_velocity_A = delta_velocity_A * inv_distance_to_A;
            delta_angular_velocity_A = Vector3.Cross(delta_angular_velocity_A, direction_to_A);
            // Settle anglular velocity when balanced on CM with normal
            if (A.r_velocity.magnitude < 0.1f && linear_percent_A > 0.9999f && relative_velocity > -2f) delta_angular_velocity_A = 0.5f * delta_angular_velocity_A - 0.5f * A.r_velocity;

            // Update linear velocities
            A.AddToAccumulator(Accumulation.Type.velocity, delta_linear_velocity_A);

            // Update angular velocities
            A.AddToAccumulator(Accumulation.Type.r_velocity, delta_angular_velocity_A);
        }

        // Undo collision in space along the normal direction
        Vector3 delta_position = Collision.disp_offset * depth * normal;
        A.AddToAccumulator(Accumulation.Type.position, delta_position);
        A.high_mass_collision = 1000;
    }

    /// <summary>
    /// Resolve collision by applying <see cref="Body.AccumulationType"/> to <see cref="Body"/>.
    /// </summary>
    /// <param name="A"></param>
    /// <param name="B"></param>
    /// <param name="position"></param>
    /// <param name="normal"></param>
    /// <param name="depth"></param>
    /// <param name="delta_time"></param>
    public static void RespondDynamicADynamicB(Body A, Body B, Vector3 position, Vector3 normal, float depth, float delta_time)
    {
        /*
         * Conservation of Momentum:
         * m1v1 + m2v2 = m1V1 + m2V2
         * m1v1 - m1V1 = m2V2 - m2v2
         * 
         * Coefficient of Restitution:
         * e = (V2 - V1) / (v1 - v2)
         * e(v1 - v2) = V2 - V1
         * V1 = -e(v1 - v2) + V2
         * V2 = e(v1 - v2) + V1
         * 
         * Solve for V1:
         * m1v1 - m1V1 = m2V2 - m2v2
         * m1v1 - m1V1 = m2(e(v1 - v2) + V1) - m2v2
         * -m1V1 = em2(v1 - v2) + m2V1 - m2v2 - m1v1
         * m1V1 = -em2(v1 - v2) - m2V1 + m2v2 + m1v1
         * m1V1 + m2V1 = -em2(v1 - v2) + m2v2 + m1v1
         * V1(m1 + m2) = -em2(v1 - v2) + m2v2 + m1v1
         * V1 = (-em2(v1 - v2) + m2v2 + m1v1) / (m1 + m2)
         * V1 = (-em2(v1 - v2) + total_momentum) / total_mass
         * 
         * Solve for V2:
         * m2V2 - m2v2 = m1v1 - m1V1
         * m2V2 - m2v2 = m1v1 - m1(-e(v1 - v2) + V2)
         * m2V2 = em1(v1 - v2) - m1V2 + m1v1 + m2v2
         * m2V2 + m1V2 = em1(v1 - v2) + m1v1 + m2v2
         * V2(m1 + m2) = em1(v1 - v2) + m1v1 + m2v2
         * V2 = (em1(v1 - v2) + m1v1 + m2v2) / (m1 + m2)
         * V2 = (em1(v1 - v2) + total_momentum) / total_mass
         * 
         * Test (perfect elastic):
         * m1, m2 = 1; e = 1; v1 = 2; v2 = 0
         * total_mass = 2; total_momentum = 2; v1 - v2 = 2
         * V1 = (-1 * 1 * 2 + 2) / 2 = 0
         * V2 = (1 * 1 * 2 + 2) / 2 = 2
         * 
         * Test (half elastic)
         * m1, m2 = 1; e = 0.5; v1 = 2; v2 = 0
         * total_mass = 2; total_momentum = 2; v1 - v2 = 2
         * V1 = (-0.5 * 1 * 2 + 2) / 2 = 0.5
         * V2 = (0.5 * 1 * 2 + 2) / 2 = 1.5
         * 
         * Test (perfect inelastic):
         * m1, m2 = 1; e = 0; v1 = 2; v2 = 0
         * total_mass = 2; total_momentum = 2; v1 - v2 = 2
         * V1 = (-0 * 1 * 2 + 2) / 2 = 1
         * V2 = (0 * 1 * 2 + 2) / 2 = 1
         */
        float friction = 0.5f * (A.friction + B.friction);
        float elasticity = 0.5f * (A.elasticity + B.elasticity);
        float total_mass = A.mass + B.mass;

        Vector3 position_to_A = A.transform.position - position;
        Vector3 position_to_B = B.transform.position - position;

        Vector3 velocity_A_from_rotation = Vector3.Cross(position_to_A, A.r_velocity);
        Vector3 velocity_B_from_rotation = Vector3.Cross(position_to_B, B.r_velocity);
        Vector3 velocity_A_at_position = A.velocity + velocity_A_from_rotation;
        Vector3 velocity_B_at_position = B.velocity + velocity_B_from_rotation;

        float acting_velocity_A = Vector3.Dot(velocity_A_at_position, normal);
        float acting_velocity_B = Vector3.Dot(velocity_B_at_position, normal);
        float relative_velocity = acting_velocity_A - acting_velocity_B;

        if (relative_velocity < 0f)
        {
            float inv_distance_to_A = 1f / position_to_A.magnitude;
            float inv_distance_to_B = 1f / position_to_B.magnitude;
            Vector3 direction_to_A = position_to_A * inv_distance_to_A;
            Vector3 direction_to_B = position_to_B * inv_distance_to_B;

            // Get distribution percents along the normal and direction to center of mass
            float linear_percent_A = Mathf.Abs(Vector3.Dot(direction_to_A, normal));
            float angular_percent_A = 1f - linear_percent_A;
            float linear_percent_B = Mathf.Abs(Vector3.Dot(direction_to_B, normal));
            float angular_percent_B = 1f - linear_percent_B;

            // Get relative moment of inertia at position along normal
            float moment_A = Vector3.Dot(A.GetRelativeMoment(), normal);
            float moment_B = Vector3.Dot(B.GetRelativeMoment(), normal);

            float reactive_velocity_A = 0f;
            float reactive_velocity_B = 0f;

            if (relative_velocity <= -2f)
            {
                // Get total momentum and mass using distributions
                float total_momentum = 0.5f * (linear_percent_A + linear_percent_B) *
                                       (A.mass * acting_velocity_A + B.mass * acting_velocity_B) +
                                       0.5f * (angular_percent_A + angular_percent_B) *
                                       (moment_A * acting_velocity_A + moment_B * acting_velocity_B);
                float total_inertial_mass = linear_percent_A * A.mass + linear_percent_B * B.mass +
                                            angular_percent_A * moment_A + angular_percent_B * moment_B;

                // Get velocity change due to impact reaction
                // (-em2(v1 - v2) + total_momentum) / total_mass
                reactive_velocity_A = -elasticity * relative_velocity *
                                            (linear_percent_B * B.mass + angular_percent_B * moment_B) +
                                            total_momentum;
                reactive_velocity_A /= total_inertial_mass;

                // (em1(v1 - v2) + total_momentum) / total_mass
                reactive_velocity_B = elasticity * relative_velocity *
                                            (linear_percent_A * A.mass + angular_percent_A * moment_A) +
                                            total_momentum;
                reactive_velocity_B /= total_inertial_mass;
            }

            // Get velocity change due to friction
            Vector3 planar_friction_A = -1f * friction * (velocity_A_at_position - acting_velocity_A * normal);
            Vector3 planar_friction_B = -1f * friction * (velocity_B_at_position - acting_velocity_B * normal);

            // Get total delta velocity required
            Vector3 delta_velocity_A = planar_friction_A + (reactive_velocity_A - Collision.react_offset * acting_velocity_A) * normal;
            Vector3 delta_velocity_B = planar_friction_B + (reactive_velocity_B - Collision.react_offset * acting_velocity_B) * normal;

            Vector3 delta_linear_velocity_A = linear_percent_A * delta_velocity_A;
            Vector3 delta_linear_velocity_B = linear_percent_B * delta_velocity_B;

            // Get angular velocity required
            Vector3 delta_angular_velocity_A = delta_velocity_A * inv_distance_to_A;
            delta_angular_velocity_A = Vector3.Cross(delta_angular_velocity_A, direction_to_A);
            // Settle anglular velocity when balanced on CM with normal
            if (A.r_velocity.magnitude < 0.1f && linear_percent_A > 0.9999f && relative_velocity > -2f) delta_angular_velocity_A = 0.5f * delta_angular_velocity_A - 0.5f * A.r_velocity;

            Vector3 delta_angular_velocity_B = delta_velocity_B * inv_distance_to_B;
            delta_angular_velocity_B = Vector3.Cross(delta_angular_velocity_B, direction_to_B);
            // Settle anglular velocity when balanced on CM with normal
            if (B.r_velocity.magnitude < 0.1f && linear_percent_B > 0.9999f && relative_velocity > -2f) delta_angular_velocity_B = 0.5f * delta_angular_velocity_B - 0.5f * B.r_velocity;

            // Update linear velocities
            A.AddToAccumulator(Accumulation.Type.velocity, delta_linear_velocity_A);
            B.AddToAccumulator(Accumulation.Type.velocity, delta_linear_velocity_B);

            // Update angular velocities
            A.AddToAccumulator(Accumulation.Type.r_velocity, delta_angular_velocity_A);
            B.AddToAccumulator(Accumulation.Type.r_velocity, delta_angular_velocity_B);
        }

        // Resolve collision spacially using depth and percent based on mass or if one object is in a rough spot (high_mass_collision)
        if (B.awake)
        {
            float percent_A = B.mass / total_mass;
            float percent_B = -1f * A.mass / total_mass;
            if (A.high_mass_collision > B.high_mass_collision)
            {
                percent_A = 0f;
                percent_B = -1f;
            }
            else if (B.high_mass_collision > A.high_mass_collision)
            {
                percent_A = 1f;
                percent_B = 0f;
            }
            Vector3 delta_position_A = percent_A * Collision.disp_offset * depth * normal;
            Vector3 delta_position_B = percent_B * Collision.disp_offset * depth * normal;
            A.AddToAccumulator(Accumulation.Type.position, delta_position_A);
            B.AddToAccumulator(Accumulation.Type.position, delta_position_B);
        }
        else
        {
            Vector3 delta_position_A = Collision.disp_offset * depth * normal;
            A.AddToAccumulator(Accumulation.Type.position, delta_position_A);
            if (A.high_mass_collision < 1) A.high_mass_collision = 10;
        }

        // Transfer high mass collision
        if (A.high_mass_collision > B.high_mass_collision) B.high_mass_collision = A.high_mass_collision - 1;
        if (B.high_mass_collision > A.high_mass_collision) A.high_mass_collision = B.high_mass_collision - 1;
    }
}
