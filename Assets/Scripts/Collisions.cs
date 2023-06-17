using System.Collections;
using System.Collections.Generic;
using UnityEngine;

public class Collision
{
    private static float disp_offset = 1.1f;
    private static float react_offset = 1.1f;

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
        OBB precise_box = new OBB(A.transform.position, A.transform.rotation, A.transform.localScale);

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
                OBB precise_box_A = new OBB(A.transform.position, A.transform.rotation, A.transform.localScale);
                OBB precise_box_B = new OBB(neighbor.transform.position, neighbor.transform.rotation, neighbor.transform.localScale);
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
    public static void ResolveImpulseDynamicAGroundB(Body A, Vector3 position, Vector3 normal, float depth, float delta_time)
    {
        float friction = A.friction;
        float elasticity = A.elasticity;

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

        Vector3 position_to_center = A.transform.position - position;
        Vector3 relative_velocity = A.velocity + Vector3.Cross(position_to_center, A.r_velocity);

        // Calculate the relative velocity between the bodies along the collision normal
        float velocity_along_normal = Vector3.Dot(relative_velocity, normal);

        // Calculate the impulse scalar based on the relative velocity
        float from_rotation = Vector3.Dot(normal, Vector3.Cross(Vector3.Cross(position_to_center, normal), Vector3.Scale(position_to_center, A.inv_moment)));
        float impulse_scalar = -(1f + elasticity) * velocity_along_normal /
                               (1f / A.mass + from_rotation);

        // Apply the impulse to the linear velocity of body A
        Vector3 impulse = impulse_scalar * normal;
        A.AddToAccumulator(Accumulation.Type.velocity, impulse / A.mass);

        // Apply friction to the linear velocity
        Vector3 friction_impulse = Vector3.zero;
        Vector3 tangent_velocity = relative_velocity - velocity_along_normal * normal;
        float tangent_velocity_mag = tangent_velocity.magnitude;

        if (tangent_velocity_mag > 0f)
        {
            Vector3 tangent_direction = tangent_velocity / tangent_velocity_mag;
            friction_impulse = -friction * impulse_scalar * tangent_direction;

            // Apply the friction impulse to the linear velocity
            float friction_impulse_mag = friction_impulse.magnitude;
            if (friction_impulse_mag < tangent_velocity_mag)
            {
                A.AddToAccumulator(Accumulation.Type.velocity, friction_impulse);
            }
            else
            {
                // If the friction impulse exceeds the tangent velocity magnitude, fully stop the tangent motion
                A.AddToAccumulator(Accumulation.Type.velocity, -tangent_velocity);
            }
        }

        // Calculate the change in angular velocity (delta_r_velocity)
        Vector3 delta_r_velocity = Quaternion.Inverse(A.transform.rotation) * Vector3.Cross(impulse + friction_impulse, position_to_center);
        delta_r_velocity = A.transform.rotation * Vector3.Scale(delta_r_velocity, A.inv_moment);
        A.AddToAccumulator(Accumulation.Type.r_velocity, delta_r_velocity);

        // Undo collision in space along the normal direction
        Vector3 delta_position = depth * normal;
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
    public static void ResolveImpulseDynamicADynamicB(Body A, Body B, Vector3 position, Vector3 normal, float depth, float delta_time)
    {
        float mass_A = A.mass;
        float mass_B = B.mass;
        float total_mass = mass_A + mass_B;
        float friction = Mathf.Sqrt(A.friction * B.friction);
        float elasticity = Mathf.Sqrt(A.elasticity * B.elasticity);

        Vector3 position_to_A = A.transform.position - position;
        Vector3 position_to_B = B.transform.position - position;
        float distance_to_A = position_to_A.magnitude;
        float distance_to_B = position_to_B.magnitude;

        Vector3 velocity_A_at_pos = A.velocity;
        Vector3 velocity_B_at_pos = B.velocity;

        // Add velocity from rotation relative to position of collision
        Vector3 velocity_from_rotation_A = Vector3.Cross(A.r_velocity, -position_to_A);
        Vector3 velocity_from_rotation_B = Vector3.Cross(B.r_velocity, -position_to_B);
        velocity_A_at_pos += velocity_from_rotation_A;
        velocity_B_at_pos += velocity_from_rotation_B;
        /*
        Vector3 velocity_from_rotation_A = Vector3.Scale(A.r_velocity, new Vector3(A.moment.x / distance_to_A, A.moment.y / distance_to_A, A.moment.z / distance_to_A));
        velocity_from_rotation_A = A.transform.rotation * velocity_from_rotation_A;
        velocity_from_rotation_A = Vector3.Cross(velocity_from_rotation_A, -position_to_A);
        velocity_A_at_pos += velocity_from_rotation_A;
        Vector3 velocity_from_rotation_B = Vector3.Scale(A.r_velocity, new Vector3(A.moment.x / distance_to_B, A.moment.y / distance_to_B, A.moment.z / distance_to_B));
        velocity_from_rotation_B = A.transform.rotation * velocity_from_rotation_B;
        velocity_from_rotation_B = Vector3.Cross(velocity_from_rotation_B, -position_to_B);
        velocity_A_at_pos += velocity_from_rotation_A;
        velocity_B_at_pos += velocity_from_rotation_B;
        */

        // Get velocities contributing and perpendicular to collision
        Vector3 acting_velocity_A = Vector3.Project(velocity_A_at_pos, normal);
        Vector3 planar_velocity_A = velocity_A_at_pos - acting_velocity_A;
        Vector3 acting_velocity_B = Vector3.Project(velocity_B_at_pos, normal);
        Vector3 planar_velocity_B = velocity_B_at_pos - acting_velocity_B;

        // Get velocity change due to friction
        Vector3 planar_friction_A = -1.0f * friction * planar_velocity_A;
        Vector3 planar_friction_B = -1.0f * friction * planar_velocity_B;

        // Get velocity change due to impact reaction
        // Get momentums in relative to collision
        Vector3 momentum_A = mass_A * acting_velocity_A;
        Vector3 momentum_B = mass_B * acting_velocity_B;
        Vector3 total_momentum = momentum_A + momentum_B;

        /*
         * (m1 * v1 + m2 * v2 + m2 * e * (v2 - v1)) / (m1 + m2)
         * (Elasticity * m2 * (v2 - v1) + Total Momentum) / Total Mass
         */
        Vector3 reactive_velocity_A = elasticity * mass_B * (acting_velocity_B - acting_velocity_A);
        reactive_velocity_A += total_momentum;
        reactive_velocity_A /= total_mass;

        /*
         * (m1 * v1 + m2 * v2 + m1 * e * (v1 - v2)) / (m1 + m2)
         * (Elasticity * m1 * (v1 - v2) + Total Momentum) / Total Mass
         */
        Vector3 reactive_velocity_B = elasticity * mass_A * (acting_velocity_A - acting_velocity_B);
        reactive_velocity_B += total_momentum;
        reactive_velocity_B /= total_mass;

        // Update velocities: planar friction + reactive
        Vector3 delta_velocity_A = planar_friction_A + reactive_velocity_A - Collision.react_offset * acting_velocity_A;
        Vector3 delta_velocity_B = planar_friction_B + reactive_velocity_B - Collision.react_offset * acting_velocity_B;
        A.AddToAccumulator(Accumulation.Type.velocity, delta_velocity_A);
        B.AddToAccumulator(Accumulation.Type.velocity, delta_velocity_B);

        // Update rotational velocities
        Vector3 relative_moment_A = A.moment;
        Vector3 torque_A = Quaternion.Inverse(A.transform.rotation) * Vector3.Cross(delta_velocity_A, position_to_A);
        Vector3 delta_r_velocity_A = A.transform.rotation * Vector3.Scale(torque_A, A.inv_moment);
        Vector3 relative_moment_B = B.moment;
        Vector3 torque_B = Quaternion.Inverse(B.transform.rotation) * Vector3.Cross(delta_velocity_B, position_to_B);
        Vector3 delta_r_velocity_B = B.transform.rotation * Vector3.Scale(torque_B, B.inv_moment);
        A.AddToAccumulator(Accumulation.Type.r_velocity, 0.5f * delta_r_velocity_A);
        B.AddToAccumulator(Accumulation.Type.r_velocity, 0.5f * delta_r_velocity_B);
        /*
        Vector3 torque_A = Vector3.Cross(delta_velocity_A, -position_to_A);
        torque_A = Quaternion.Inverse(A.transform.rotation) * torque_A;
        Vector3 torque_B = Vector3.Cross(delta_velocity_B, -position_to_B);
        torque_B = Quaternion.Inverse(B.transform.rotation) * torque_B;
        Vector3 delta_r_velocity_A = Vector3.Scale(torque_A, new Vector3(distance_to_A / A.moment.x, distance_to_A / A.moment.y, distance_to_A / A.moment.z));
        Vector3 delta_r_velocity_B = Vector3.Scale(torque_B, new Vector3(distance_to_B / B.moment.x, distance_to_B / B.moment.y, distance_to_B / B.moment.z));
        A.AddToAccumulator(Accumulation.Type.r_velocity, delta_r_velocity_A);
        B.AddToAccumulator(Accumulation.Type.r_velocity, delta_r_velocity_B);
        */

        // Resolve collision spacially using depth and percent based on mass or if one object is in a rough spot (high_mass_collision)
        if (B.awake)
        {
            float percent_A = mass_B / total_mass;
            float percent_B = -1f * mass_A / total_mass;
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
            Vector3 delta_position_A = percent_A * 0.5f * Collision.disp_offset * depth * normal;
            Vector3 delta_position_B = percent_B * 0.5f * Collision.disp_offset * depth * normal;
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
