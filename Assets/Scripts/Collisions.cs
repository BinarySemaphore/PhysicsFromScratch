using System.Collections;
using System.Collections.Generic;
using UnityEngine;

public class Collision
{
    public Body A;
    public Body B;
    public Vector3 location;
    public Vector3 normal;

    /// <summary>
    /// Collision information.
    /// </summary>
    /// <param name="A"></param>
    /// <param name="B"></param>
    /// <param name="location"></param>
    /// <param name="normal"></param>
    public Collision(Body A, Body B, Vector3 location, Vector3 normal)
    {
        this.A = A;
        this.B = B;
        this.location = location;
        this.normal = normal;
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
            if (AABB.IsOverlapping(A, neighbor))
            {
                AABB precise_box_A = new AABB(A.entity, AABB.Containment.smallest);
                AABB precise_box_B = new AABB(neighbor.entity, AABB.Containment.smallest);
                if (AABB.IsOverlapping(precise_box_A, precise_box_B))
                {
                    collisions.Add(new Collision(
                        A, neighbor,
                        A.center + 0.5f * (neighbor.center - A.center),
                        AABB.NormalOfOverlapping(precise_box_A, precise_box_B)
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
    /// <param name="B"></param>
    /// <param name="position"></param>
    /// <param name="normal"></param>
    /// <param name="detla_time"></param>
    public static void ResolveImpulseDynamicADynamicB(Body A, Body B, Vector3 position, Vector3 normal, float delta_time)
    {
        float total_mass = A.mass + B.mass;
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
        Vector3 momentum_A = A.mass * acting_velocity_A;
        Vector3 momentum_B = B.mass * acting_velocity_B;
        Vector3 total_momentum = momentum_A + momentum_B;

        /*
         * (m1 * v1 + m2 * v2 + m2 * e * (v2 - v1)) / (m1 + m2)
         * (Elasticity * m2 * (v2 - v1) + Total Momentum) / Total Mass
         */
        Vector3 reactive_velocity_A = elasticity * B.mass * (acting_velocity_B - acting_velocity_A);
        reactive_velocity_A += total_momentum;
        reactive_velocity_A = reactive_velocity_A / total_mass;

        /*
         * (m1 * v1 + m2 * v2 + m1 * e * (v1 - v2)) / (m1 + m2)
         * (Elasticity * m1 * (v1 - v2) + Total KE) / Total Mass
         */
        Vector3 reactive_velocity_B = elasticity * A.mass * (acting_velocity_A - acting_velocity_B);
        reactive_velocity_B += total_momentum;
        reactive_velocity_B = reactive_velocity_B / total_mass;

        // Update velocities: planar friction + reactive
        Vector3 delta_velocity_A = planar_friction_A + reactive_velocity_A - acting_velocity_A;
        Vector3 delta_velocity_B = planar_friction_B + reactive_velocity_B - acting_velocity_B;
        A.AddToAccumulator(Accumulation.Type.velocity, delta_velocity_A);
        B.AddToAccumulator(Accumulation.Type.velocity, delta_velocity_B);

        // Undo collision in space along normal for most energetic body(s)
        Vector3 delta_position_A = -1.0f * delta_time * acting_velocity_A;
        Vector3 delta_position_B = -1.0f * delta_time * acting_velocity_B;
        if (acting_velocity_A.magnitude > acting_velocity_B.magnitude)
        {
            A.AddToAccumulator(Accumulation.Type.position, delta_position_A);
        }
        else if (acting_velocity_B.magnitude > acting_velocity_A.magnitude)
        {
            B.AddToAccumulator(Accumulation.Type.position, delta_position_B);
        }
        else
        {
            A.AddToAccumulator(Accumulation.Type.position, delta_position_A);
            B.AddToAccumulator(Accumulation.Type.position, delta_position_B);
        }
    }
}
