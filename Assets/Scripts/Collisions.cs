using System.Collections.Generic;
using UnityEngine;

public class Collision
{
    private static readonly float disp_offset = 1.0f;
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

        //Vector3 position_to_A = A.transform.position - position;

        //// Get total velocities at point
        //Vector3 velocity_A_from_rotation = Vector3.Cross(position_to_A, A.r_velocity);
        //Vector3 velocity_A_at_pos = A.velocity + velocity_A_from_rotation;
        //Vector3 relative_velocity = velocity_A_at_pos;

        //// Check if collision response required
        //if (Vector3.Dot(relative_velocity, normal) >= 0) return;

        //// Get velocity change due to impact reaction
        //// Get momentums in relative to collision
        //Vector3 momentum_A = A.mass * A.velocity + Vector3.Scale(velocity_A_from_rotation, A.moment);
        //Vector3 total_momentum = momentum_A;

        ///*
        // * (-em2(v1 - v2) + m2v2 + m1v1) / (m1 + m2)
        // * (-em2(v1 - v2) + total_momentum) / total_mass
        // */
        //Vector3 final_velocity_A = -elasticity * 1000000f * Vector3.Project(relative_velocity + total_momentum, normal);
        //final_velocity_A /= total_mass;

        //Vector3 delta_v_A_at_position = final_velocity_A - Vector3.Project(velocity_A_at_pos, normal);

        //// Distribute delta v between linear (including friction) and angular velocities
        //float linear_percent_A = Mathf.Abs(Vector3.Dot(position_to_A.normalized, normal));
        //Vector3 linear_velocity_A = linear_percent_A * delta_v_A_at_position;
        //Vector3 delta_linear_v_A = linear_velocity_A;//Vector3.Project(linear_velocity_A, normal);

        //// Find friction and apply to delta linear v's
        //delta_linear_v_A -= friction * (velocity_A_at_pos - Vector3.Project(velocity_A_at_pos, normal));

        //// Remaining velocity to rotation L = Iw = mvR
        //// w = mvRi
        //Vector3 delta_angular_v_A = delta_linear_v_A / position_to_A.magnitude;
        //delta_angular_v_A = Vector3.Cross(delta_angular_v_A, position_to_A.normalized);
        ////delta_angular_v_A = A.transform.rotation * Vector3.Scale(delta_angular_v_A, A.inv_moment);
        ////delta_angular_v_A = delta_angular_v_A - 0.5f * A.r_velocity;

        //// Update velocities:
        //A.AddToAccumulator(Accumulation.Type.velocity, delta_linear_v_A);

        //// Update rotational velocities
        //A.AddToAccumulator(Accumulation.Type.r_velocity, delta_angular_v_A);

        Vector3 position_to_A = A.transform.position - position;
        float inv_distance_to_A = 1f / position_to_A.magnitude;
        Vector3 direction_to_A = position_to_A * inv_distance_to_A;

        Vector3 velocity_A_from_rotation = Vector3.Cross(position_to_A, A.r_velocity);
        Vector3 velocity_A_at_position = A.velocity + velocity_A_from_rotation;
        float acting_velocity_A = Vector3.Dot(velocity_A_at_position, normal);
        float relative_velocity = acting_velocity_A;  // Normally subtract acting_velocity_B but B is static

        //Vector3 acting_velocity_A = Vector3.Project(velocity_A_at_position, normal);
        //Vector3 planar_velocity_A = velocity_A_at_position - acting_velocity_A;
        //Vector3 acting_relative_velocity = acting_velocity_A - Vector3.zero;

        if (relative_velocity > -2f) elasticity = 0f;

        if (relative_velocity < 0f)
        {
            //Get velocity change due to impact reaction
            // -e(v1 - v2) / m1  <- as m2 is infinite mass, m2 is cancelled from multiplying by division and addition of acting_total_momentum aproaches zero so it is ignored.
            float reactive_velocity_A = -elasticity * relative_velocity;
            reactive_velocity_A /= A.mass;

            //Get velocity change due to friction
            Vector3 planar_friction_A = -1f * friction * (velocity_A_at_position - acting_velocity_A * normal);

            //Get total delta velocity required
            Vector3 delta_velocity_A = planar_friction_A + (reactive_velocity_A - Collision.react_offset * acting_velocity_A) * normal;

            float linear_percent_A = Mathf.Abs(Vector3.Dot(direction_to_A, normal));
            Vector3 delta_linear_velocity_A = linear_percent_A * delta_velocity_A;

            Vector3 delta_angular_velocity_A = delta_velocity_A * inv_distance_to_A;
            delta_angular_velocity_A = Vector3.Cross(delta_angular_velocity_A, direction_to_A);
            //if (relative_velocity > -2f && linear_percent_A > 0.99f) delta_angular_velocity_A = (relative_velocity / -2f) * delta_angular_velocity_A - (relative_velocity / -2f) * A.r_velocity;
            //if (linear_percent_A > 0.999999f) delta_angular_velocity_A = 0.5f * delta_angular_velocity_A - 0.5f * A.r_velocity; // Settle anglular velocity when balanced on CM with normal
            //if (linear_percent_A > 0.9999f) delta_angular_velocity_A = 0.7f * delta_angular_velocity_A - 0.3f * A.r_velocity; // Settle anglular velocity when balanced on CM with normal
            //if (linear_percent_A > 0.99f) delta_angular_velocity_A = 0.9f * delta_angular_velocity_A - 0.1f * A.r_velocity; // Settle anglular velocity when balanced on CM with normal

            A.AddToAccumulator(Accumulation.Type.velocity, delta_linear_velocity_A);
            A.AddToAccumulator(Accumulation.Type.r_velocity, delta_angular_velocity_A);
        }

        //float mass_A = A.mass;
        //float mass_B = 100000f;
        //float total_mass = mass_A + mass_B;
        //Vector3 position_to_A = A.transform.position - position;
        //float distance_to_A = position_to_A.magnitude;

        //Vector3 velocity_A_at_pos = A.velocity + Vector3.Cross(A.r_velocity, -position_to_A);

        //// Get velocities contributing and perpendicular to collision
        //Vector3 acting_velocity_A = Vector3.Project(velocity_A_at_pos, normal);
        //Vector3 planar_velocity_A = velocity_A_at_pos - acting_velocity_A;

        //// Get velocity change due to friction
        //Vector3 planar_friction_A = -1.0f * friction * planar_velocity_A;

        //// Get velocity change due to impact reaction
        //// Get momentums in relative to collision
        //Vector3 momentum_A = mass_A * acting_velocity_A;
        //Vector3 total_momentum = momentum_A;

        ///*
        // * (m1 * v1 + m2 * v2 + m2 * e * (v2 - v1)) / (m1 + m2)
        // * (Elasticity * m2 * (v2 - v1) + Total Momentum) / Total Mass
        // */
        //float from_rotation = Vector3.Dot(normal, Vector3.Cross(Vector3.Cross(normal, position_to_A), Vector3.Scale(position_to_A, A.inv_moment)));
        //Vector3 reactive_velocity_A = elasticity * mass_B * (Vector3.zero - acting_velocity_A);
        //reactive_velocity_A += total_momentum;
        //reactive_velocity_A /= total_mass + from_rotation;

        //// Update velocities: planar friction + reactive
        //Vector3 delta_velocity_A = planar_friction_A + reactive_velocity_A - acting_velocity_A;
        //A.AddToAccumulator(Accumulation.Type.velocity, delta_velocity_A);

        //// Update rotational velocities
        //Vector3 delta_r_velocity_A = Quaternion.Inverse(A.transform.rotation) * Vector3.Cross(0.25f * delta_velocity_A, position_to_A);
        //delta_r_velocity_A = A.transform.rotation * Vector3.Scale(delta_r_velocity_A, A.inv_moment);
        //A.AddToAccumulator(Accumulation.Type.r_velocity, delta_r_velocity_A);

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
    /// <param name="detla_time"></param>
    public static void RespondDynamicADynamicB(Body A, Body B, Vector3 position, Vector3 normal, float depth, float delta_time)
    {
        //Use this
        /*
        Vector3 position_to_A = A.transform.position - position;

        Vector3 velocity_A_from_rotation = Vector3.Cross(position_to_A, A.r_velocity);
        Vector3 velocity_A_at_position = A.velocity + velocity_A_from_rotation;

        Vector3 acting_velocity_A = Vector3.Project(velocity_A_at_position, normal);
        Vector3 planar_velocity_A = velocity_A_at_position - acting_velocity_A;
        Vector3 acting_relative_velocity = acting_velocity_A - Vector3.zero;

        if (Vector3.Dot(acting_relative_velocity, normal) < 0)
        {
            Vector3 planar_friction_A = -1f * friction * planar_velocity_A;

            // In this case total_momentum will be divided by infiinity so  can ingnore after testing (keep around for copy+paste)
            Vector3 momentum_A = A.mass * A.velocity + Vector3.Scale(velocity_A_from_rotation, A.moment);
            Vector3 total_momentum = momentum_A;
            Vector3 acting_total_momentum = Vector3.Project(total_momentum, normal);

            // (-em2(v1 - v2) + total_momentum) / total_mass
            Vector3 reactive_velocity_A = -elasticity * 1000000f * acting_relative_velocity + acting_total_momentum;
            reactive_velocity_A /= total_mass;

            Vector3 delta_velocity_A = planar_friction_A + reactive_velocity_A - acting_velocity_A;

            float linear_percent_A = Mathf.Abs(Vector3.Dot(position_to_A.normalized, normal));
            Vector3 delta_linear_velocity_A = linear_percent_A * delta_velocity_A;

            Vector3 delta_angular_velocity_A = delta_linear_velocity_A / position_to_A.magnitude;
            delta_angular_velocity_A = Vector3.Cross(delta_angular_velocity_A, position_to_A.normalized);

            A.AddToAccumulator(Accumulation.Type.velocity, delta_linear_velocity_A);
            A.AddToAccumulator(Accumulation.Type.r_velocity, delta_angular_velocity_A);
        }
        */
        float friction = 0.5f * (A.friction + B.friction);
        float elasticity = 0.5f * (A.elasticity + B.elasticity);
        float total_mass = A.mass + B.mass;

        Vector3 position_to_A = A.transform.position - position;
        Vector3 position_to_B = B.transform.position - position;

        Vector3 velocity_A_at_position = A.velocity;
        Vector3 velocity_B_at_position = B.velocity;

        float acting_velocity_A = Vector3.Dot(velocity_A_at_position, normal);
        float acting_velocity_B = Vector3.Dot(velocity_B_at_position, normal);
        float relative_velocity = acting_velocity_A - acting_velocity_B;

        if (relative_velocity > -2f) elasticity = 0f;

        if (relative_velocity < 0f)
        {
            float inv_distance_to_A = 1f / position_to_A.magnitude;
            float inv_distance_to_B = 1f / position_to_B.magnitude;
            Vector3 direction_to_A = position_to_A * inv_distance_to_A;
            Vector3 direction_to_B = position_to_B * inv_distance_to_B;

            float total_momentum = A.mass * acting_velocity_A +
                                   B.mass * acting_velocity_B;

            //Get velocity change due to impact reaction
            // (-em2(v1 - v2) + total_momentum) / total_mass
            float reactive_velocity_A = -elasticity * B.mass * relative_velocity + total_momentum;
            reactive_velocity_A /= total_mass;

            // (em1(v1 - v2) + total_momentum) / total_mass
            float reactive_velocity_B = elasticity * A.mass * relative_velocity + total_momentum;
            reactive_velocity_B /= total_mass;

            //Get velocity change due to friction
            Vector3 planar_friction_A = -1f * friction * (velocity_A_at_position - acting_velocity_A * normal);
            Vector3 planar_friction_B = -1f * friction * (velocity_B_at_position - acting_velocity_B * normal);

            //Get total delta velocity required
            Vector3 delta_velocity_A = planar_friction_A + (reactive_velocity_A - Collision.react_offset * acting_velocity_A) * normal;
            Vector3 delta_velocity_B = planar_friction_B + (reactive_velocity_B - Collision.react_offset * acting_velocity_B) * normal;

            float linear_percent_A = Mathf.Abs(Vector3.Dot(direction_to_A, normal));
            float linear_percent_B = Mathf.Abs(Vector3.Dot(direction_to_B, normal));
            Vector3 delta_linear_velocity_A = linear_percent_A * delta_velocity_A;
            Vector3 delta_linear_velocity_B = linear_percent_B * delta_velocity_B;

            Vector3 delta_angular_velocity_A = delta_linear_velocity_A * inv_distance_to_A;
            delta_angular_velocity_A = Vector3.Cross(delta_angular_velocity_A, direction_to_A);
            //if (linear_percent_A > 0.999999f) delta_angular_velocity_A = 0.5f * delta_angular_velocity_A - 0.5f * A.r_velocity; // Settle anglular velocity when balanced on CM with normal

            Vector3 delta_angular_velocity_B = delta_linear_velocity_B * inv_distance_to_B;
            delta_angular_velocity_B = Vector3.Cross(delta_angular_velocity_B, direction_to_B);
            //if (linear_percent_B > 0.999999f) delta_angular_velocity_B = 0.5f * delta_angular_velocity_B - 0.5f * B.r_velocity; // Settle anglular velocity when balanced on CM with normal

            // Update linear velocities
            A.AddToAccumulator(Accumulation.Type.velocity, delta_linear_velocity_A);
            B.AddToAccumulator(Accumulation.Type.velocity, delta_linear_velocity_B);

            // Update angular velocities
            A.AddToAccumulator(Accumulation.Type.r_velocity, delta_angular_velocity_A);
            B.AddToAccumulator(Accumulation.Type.r_velocity, delta_angular_velocity_B);
        }

        Vector3 relative_moment_A = A.transform.rotation * A.moment;
        Vector3 relative_moment_B = B.transform.rotation * B.moment;
        relative_moment_A.x = Mathf.Abs(relative_moment_A.x);
        relative_moment_A.y = Mathf.Abs(relative_moment_A.y);
        relative_moment_A.z = Mathf.Abs(relative_moment_A.z);
        relative_moment_B.x = Mathf.Abs(relative_moment_B.x);
        relative_moment_B.y = Mathf.Abs(relative_moment_B.y);
        relative_moment_B.z = Mathf.Abs(relative_moment_B.z);
        Vector3 velocity_A_from_rotation = Vector3.Cross(position_to_A, A.r_velocity);
        Vector3 velocity_B_from_rotation = Vector3.Cross(position_to_B, B.r_velocity);
        float acting_velocity_A2 = Vector3.Dot(velocity_A_from_rotation, normal);
        float acting_velocity_B2 = Vector3.Dot(velocity_B_from_rotation, normal);
        float relative_velocity2 = acting_velocity_A2 - acting_velocity_B2;

        if (relative_velocity2 < 0f)
        {
            float inv_distance_to_A = 1f / position_to_A.magnitude;
            float inv_distance_to_B = 1f / position_to_B.magnitude;
            Vector3 direction_to_A = position_to_A * inv_distance_to_A;
            Vector3 direction_to_B = position_to_B * inv_distance_to_B;

            float total_momentum = Vector3.Dot(
                Vector3.Scale(velocity_A_from_rotation, relative_moment_A) +
                Vector3.Scale(velocity_B_from_rotation, relative_moment_B),
                normal);

            //Get velocity change due to impact reaction
            // (-em2(v1 - v2) + total_momentum) / total_mass
            float reactive_velocity_A = -elasticity * Vector3.Dot(relative_moment_B, normal) * relative_velocity2 + total_momentum;
            reactive_velocity_A /= Vector3.Dot(relative_moment_A + relative_moment_B, normal);

            // (em1(v1 - v2) + total_momentum) / total_mass
            float reactive_velocity_B = elasticity * Vector3.Dot(relative_moment_A, normal) * relative_velocity2 + total_momentum;
            reactive_velocity_B /= Vector3.Dot(relative_moment_A + relative_moment_B, normal);

            //Get velocity change due to friction
            Vector3 planar_friction_A = -1f * friction * (velocity_A_from_rotation - acting_velocity_A2 * normal);
            Vector3 planar_friction_B = -1f * friction * (velocity_B_from_rotation - acting_velocity_B2 * normal);

            //Get total delta velocity required
            Vector3 delta_velocity_A = planar_friction_A + (reactive_velocity_A - Collision.react_offset * acting_velocity_A2) * normal;
            Vector3 delta_velocity_B = planar_friction_B + (reactive_velocity_B - Collision.react_offset * acting_velocity_B2) * normal;

            float linear_percent_A = Mathf.Abs(Vector3.Dot(direction_to_A, normal));
            float linear_percent_B = Mathf.Abs(Vector3.Dot(direction_to_B, normal));
            Vector3 delta_linear_velocity_A = linear_percent_A * delta_velocity_A;
            Vector3 delta_linear_velocity_B = linear_percent_B * delta_velocity_B;

            Vector3 delta_angular_velocity_A = delta_linear_velocity_A * inv_distance_to_A;
            delta_angular_velocity_A = Vector3.Cross(delta_angular_velocity_A, direction_to_A);
            //if (linear_percent_A > 0.999999f) delta_angular_velocity_A = 0.5f * delta_angular_velocity_A - 0.5f * A.r_velocity; // Settle anglular velocity when balanced on CM with normal

            Vector3 delta_angular_velocity_B = delta_linear_velocity_B * inv_distance_to_B;
            delta_angular_velocity_B = Vector3.Cross(delta_angular_velocity_B, direction_to_B);
            //if (linear_percent_B > 0.999999f) delta_angular_velocity_B = 0.5f * delta_angular_velocity_B - 0.5f * B.r_velocity; // Settle anglular velocity when balanced on CM with normal

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

    public static void RespondCombinedDynamicADynamicB(Body A, Body B, Vector3 position, Vector3 normal, float depth, float delta_time)
    {
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

        if (relative_velocity > -2f) elasticity = 0f;

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

            // Get total momentum and mass using distributions
            float total_momentum = 0.5f * (linear_percent_A + linear_percent_B) * (A.mass * acting_velocity_A + B.mass * acting_velocity_B) +
                                   0.5f * (angular_percent_A + angular_percent_B) * (moment_A * acting_velocity_A + moment_B * acting_velocity_B);
            float total_inertial_mass = linear_percent_A * A.mass + linear_percent_B * B.mass + angular_percent_A * moment_A + angular_percent_B * moment_B;

            //Get velocity change due to impact reaction
            // (-em2(v1 - v2) + total_momentum) / total_mass
            float reactive_velocity_A = -elasticity * relative_velocity * (linear_percent_B * B.mass + angular_percent_B * moment_B) + total_momentum;
            reactive_velocity_A /= total_inertial_mass;

            // (em1(v1 - v2) + total_momentum) / total_mass
            float reactive_velocity_B = elasticity * relative_velocity * (linear_percent_A * A.mass + angular_percent_A * moment_A) + total_momentum;
            reactive_velocity_B /= total_inertial_mass;

            //Get velocity change due to friction
            Vector3 planar_friction_A = -1f * friction * (velocity_A_at_position - acting_velocity_A * normal);
            Vector3 planar_friction_B = -1f * friction * (velocity_B_at_position - acting_velocity_B * normal);

            //Get total delta velocity required
            Vector3 delta_velocity_A = planar_friction_A + (reactive_velocity_A - Collision.react_offset * acting_velocity_A) * normal;
            Vector3 delta_velocity_B = planar_friction_B + (reactive_velocity_B - Collision.react_offset * acting_velocity_B) * normal;

            Vector3 delta_linear_velocity_A = linear_percent_A * delta_velocity_A;
            Vector3 delta_linear_velocity_B = linear_percent_B * delta_velocity_B;

            Vector3 delta_angular_velocity_A = delta_linear_velocity_A * inv_distance_to_A;
            delta_angular_velocity_A = Vector3.Cross(delta_angular_velocity_A, direction_to_A);
            //if (linear_percent_A > 0.999999f) delta_angular_velocity_A = 0.5f * delta_angular_velocity_A - 0.5f * A.r_velocity; // Settle anglular velocity when balanced on CM with normal

            Vector3 delta_angular_velocity_B = delta_linear_velocity_B * inv_distance_to_B;
            delta_angular_velocity_B = Vector3.Cross(delta_angular_velocity_B, direction_to_B);
            //if (linear_percent_B > 0.999999f) delta_angular_velocity_B = 0.5f * delta_angular_velocity_B - 0.5f * B.r_velocity; // Settle anglular velocity when balanced on CM with normal

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

    public static void RespondHeckerDynamicAGroundB(Body A, Vector3 position, Vector3 normal, float depth, float delta_time)
    {
        float friction = A.friction;
        float elasticity = A.elasticity;

        Vector3 position_to_A = A.transform.position - position;
        Vector3 relative_velocity = A.velocity + Vector3.Cross(position_to_A, A.r_velocity);

        // Calculate the relative velocity between the bodies along the collision normal
        float velocity_along_normal = Vector3.Dot(relative_velocity, normal);

        // Check if moving away, ignore if true
        if (velocity_along_normal >= 0f) return;

        // Calculate the impulse scalar based on the relative velocity
        float angular_moment_A = Vector3.Dot(normal, Vector3.Cross(Vector3.Scale(Vector3.Cross(position_to_A, normal), A.inv_moment), position_to_A));
        float impulse_mag = -(1f + elasticity) * velocity_along_normal /
                            (A.inv_mass + angular_moment_A);
        Vector3 impulse_A = impulse_mag * normal;

        // Apply the impulse to the linear velocity of body A
        A.AddToAccumulator(Accumulation.Type.velocity, impulse_A * A.inv_mass);

        // Apply friction to the linear velocity
        Vector3 friction_impulse = Vector3.zero;
        Vector3 tangent_velocity = relative_velocity - velocity_along_normal * normal;
        float tangent_velocity_mag = tangent_velocity.magnitude;

        if (tangent_velocity_mag > 0f)
        {
            Vector3 tangent_direction = tangent_velocity / tangent_velocity_mag;
            friction_impulse = -friction * impulse_mag * tangent_direction;

            // Apply the friction impulse to the linear velocity
            float friction_impulse_mag = friction_impulse.magnitude;
            if (friction_impulse_mag < tangent_velocity_mag)
            {
                A.AddToAccumulator(Accumulation.Type.velocity, friction_impulse * A.inv_mass);
            }
            else
            {
                // If the friction impulse exceeds the tangent velocity magnitude, fully stop the tangent motion
                A.AddToAccumulator(Accumulation.Type.velocity, -tangent_velocity);
            }
        }

        // Calculate the change in angular velocity (delta_r_velocity)
        Vector3 delta_r_velocity = Quaternion.Inverse(A.transform.rotation) * Vector3.Cross(impulse_A + friction_impulse, position_to_A);
        delta_r_velocity = A.transform.rotation * Vector3.Scale(delta_r_velocity, A.inv_moment);
        A.AddToAccumulator(Accumulation.Type.r_velocity, delta_r_velocity);

        // Undo collision in space along the normal direction
        Vector3 delta_position = Collision.disp_offset * depth * normal;
        A.AddToAccumulator(Accumulation.Type.position, delta_position);
        A.high_mass_collision = 1000;
    }



    public static void RespondHeckerDynamicADynamicB(Body A, Body B, Vector3 position, Vector3 normal, float depth, float delta_time)
    {
        float friction = 0.5f * (A.friction + B.friction);
        float elasticity = 0.5f * (A.elasticity + B.elasticity);
        float total_mass = A.mass + B.mass;

        Vector3 position_to_A = A.transform.position - position;
        Vector3 position_to_B = B.transform.position - position;
        Vector3 relative_velocity = A.velocity + Vector3.Cross(position_to_A, A.r_velocity) - B.velocity - Vector3.Cross(position_to_B, B.r_velocity);

        // Calculate the relative velocity between the bodies along the collision normal
        float velocity_along_normal = Vector3.Dot(relative_velocity, normal);

        // Check if moving away, ignore if true
        if (velocity_along_normal >= 0f) return;

        // Calculate the impulse scalar based on the relative velocity
        Vector3 angular_moment_A = Vector3.Cross(Vector3.Scale(Vector3.Cross(position_to_A, normal), A.inv_moment), position_to_A);
        Vector3 angular_moment_B = Vector3.Cross(Vector3.Scale(Vector3.Cross(position_to_B, normal), B.inv_moment), position_to_B);
        float total_angular_moment = Vector3.Dot(normal, angular_moment_A + angular_moment_B);
        float impulse_mag = -(1f + elasticity) * velocity_along_normal /
                            (A.inv_mass + B.inv_mass + total_angular_moment);
        Vector3 impulse_A = impulse_mag * normal;
        Vector3 impulse_B = -impulse_mag * normal;

        // Apply the impulse to the linear velocity of body A
        A.AddToAccumulator(Accumulation.Type.velocity, impulse_A * A.inv_mass);
        B.AddToAccumulator(Accumulation.Type.velocity, impulse_B * B.inv_mass);

        // Apply friction to the linear velocity
        Vector3 friction_impulse_A = Vector3.zero;
        Vector3 friction_impulse_B = Vector3.zero;
        Vector3 tangent_velocity = relative_velocity - velocity_along_normal * normal;
        float tangent_velocity_mag = tangent_velocity.magnitude;

        if (tangent_velocity_mag > 0f)
        {
            Vector3 tangent_direction = tangent_velocity / tangent_velocity_mag;
            friction_impulse_A = -friction * impulse_mag * tangent_direction;
            friction_impulse_B = -friction * impulse_mag * tangent_direction;

            // Apply the friction impulse to the linear velocity
            if (friction_impulse_A.magnitude < tangent_velocity_mag)
            {
                A.AddToAccumulator(Accumulation.Type.velocity, friction_impulse_A * A.inv_mass);
            }
            else
            {
                // If the friction impulse exceeds the tangent velocity magnitude, fully stop the tangent motion
                A.AddToAccumulator(Accumulation.Type.velocity, -tangent_velocity);
            }
            if (friction_impulse_B.magnitude < tangent_velocity_mag)
            {
                B.AddToAccumulator(Accumulation.Type.velocity, friction_impulse_B * B.inv_mass);
            }
            else
            {
                B.AddToAccumulator(Accumulation.Type.velocity, tangent_velocity);
            }
        }

        // Calculate the change in angular velocity (delta_r_velocity)
        Vector3 delta_r_velocity_A = Quaternion.Inverse(A.transform.rotation) * Vector3.Cross(impulse_A + friction_impulse_A, position_to_A);
        delta_r_velocity_A = A.transform.rotation * Vector3.Scale(delta_r_velocity_A, A.inv_moment);
        Vector3 delta_r_velocity_B = Quaternion.Inverse(B.transform.rotation) * Vector3.Cross(impulse_B + friction_impulse_B, position_to_B);
        delta_r_velocity_B = B.transform.rotation * Vector3.Scale(delta_r_velocity_B, B.inv_moment);
        A.AddToAccumulator(Accumulation.Type.r_velocity, delta_r_velocity_A);
        B.AddToAccumulator(Accumulation.Type.r_velocity, delta_r_velocity_B);

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
