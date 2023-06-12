using System.Collections;
using System.Collections.Generic;
using UnityEngine;

public class Collision
{
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
        Vector3 origin = A.transform.position;
        origin.y += dims_y + velocity_down;
        float distance = 2 * (dims_y + velocity_down);
        if (Physics.Raycast(origin, Vector3.down, out RaycastHit hit, distance))
        {
            if (hit.collider.tag == A.sim.grounds_tag)
            {
                float depth = hit.point.y - (A.transform.position.y - dims_y);
                collisions.Add(new Collision(
                    A, null,
                    hit.point,
                    hit.normal,
                    depth
                ));
            }
        }
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
                float depth;
                Vector3 location;
                Vector3 normal;
                OBB precise_box_A = new OBB(A.transform.position, A.transform.rotation, A.transform.localScale);
                OBB precise_box_B = new OBB(neighbor.transform.position, neighbor.transform.rotation, neighbor.transform.localScale);
                if (OBB.IsOverlapping(precise_box_A, precise_box_B, out location, out normal, out depth))
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

        Vector3 acting_velocity = Vector3.Project(A.velocity, normal);
        Vector3 planar_velocity = A.velocity - acting_velocity;

        // Get velocity change due to friction
        Vector3 planar_friction = -1.0f * friction * planar_velocity;

        // Get velocity change due to impact reaction
        Vector3 reactive_velocity = -1.0f * elasticity * acting_velocity;

        // Update velocities: planar friction + reactive - acting
        Vector3 delta_velocity = planar_friction + reactive_velocity - acting_velocity;
        A.AddToAccumulator(Accumulation.Type.velocity, delta_velocity);

        // Undo collision in space along normal
        Vector3 delta_position = new Vector3(0f, depth, 0f);
        //A.transform.position += delta_position;
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

        Vector3 acting_velocity_A = Vector3.Project(A.velocity, normal);
        Vector3 planar_velocity_A = A.velocity - acting_velocity_A;
        Vector3 acting_velocity_B = Vector3.Project(B.velocity, normal);
        Vector3 planar_velocity_B = B.velocity - acting_velocity_B;

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
        reactive_velocity_A = reactive_velocity_A / total_mass;

        /*
         * (m1 * v1 + m2 * v2 + m1 * e * (v1 - v2)) / (m1 + m2)
         * (Elasticity * m1 * (v1 - v2) + Total KE) / Total Mass
         */
        Vector3 reactive_velocity_B = elasticity * mass_A * (acting_velocity_A - acting_velocity_B);
        reactive_velocity_B += total_momentum;
        reactive_velocity_B = reactive_velocity_B / total_mass;

        // Update velocities: planar friction + reactive
        Vector3 delta_velocity_A = planar_friction_A + reactive_velocity_A - 1.1f * acting_velocity_A;
        Vector3 delta_velocity_B = planar_friction_B + reactive_velocity_B - 1.1f * acting_velocity_B;
        A.AddToAccumulator(Accumulation.Type.velocity, delta_velocity_A);
        B.AddToAccumulator(Accumulation.Type.velocity, delta_velocity_B);

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
            Vector3 delta_position_A = percent_A * 1.1f * depth * normal;
            Vector3 delta_position_B = percent_B * 1.1f * depth * normal;
            A.AddToAccumulator(Accumulation.Type.position, delta_position_A);
            B.AddToAccumulator(Accumulation.Type.position, delta_position_B);
        }
        else
        {
            Vector3 delta_position_A = depth * normal;
            A.AddToAccumulator(Accumulation.Type.position, delta_position_A);
            if (A.high_mass_collision < 1) A.high_mass_collision = 10;
        }

        // Transfer high mass collision
        if (A.high_mass_collision > B.high_mass_collision) B.high_mass_collision = A.high_mass_collision - 1;
        if (B.high_mass_collision > A.high_mass_collision) A.high_mass_collision = B.high_mass_collision - 1;
    }
}
