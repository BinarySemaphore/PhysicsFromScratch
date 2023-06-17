using System.Collections;
using System.Collections.Generic;
using UnityEngine;

public class Accumulation
{
    public enum Type
    {
        position = 0,
        velocity = 1,
        r_velocity = 2
    }

    public Type type;
    public Vector3 value;

    public Accumulation(Type type, Vector3 value)
    {
        this.type = type;
        this.value = value;
    }
}

/// <summary>
/// Physics Body
/// </summary>
public class Body : MonoBehaviour
{
    public Vector3 inv_moment;

    public bool apply_gravity;
    public bool awake;
    public int high_mass_collision;
    public float idle_time;
    public float mass;
    public float friction;
    public float elasticity;
    public Vector3 moment;
    public Vector3 velocity;
    public Vector3 r_velocity;
    public Vector3 last_postion;
    public Vector3 last_rotation;
    public OctreeItem bounding_box;
    public List<Accumulation> accumulator;
    public Simulator sim;

    // Start is called before the first frame update
    void Start()
    {
        this.awake = true;
        this.high_mass_collision = 0;
        this.idle_time = 0;
        /*
         * https://en.wikipedia.org/wiki/List_of_moments_of_inertia
         * This should be 1/12 * mass * (width^2 + height^2) but 1/20 seems to look better.
         */
        this.moment = new Vector3(
            0.0833f * this.mass * (this.transform.localScale.y * this.transform.localScale.y + this.transform.localScale.z * this.transform.localScale.z),
            0.0833f * this.mass * (this.transform.localScale.x * this.transform.localScale.x + this.transform.localScale.z * this.transform.localScale.z),
            0.0833f * this.mass * (this.transform.localScale.x * this.transform.localScale.x + this.transform.localScale.y * this.transform.localScale.y)
        );
        this.inv_moment = new Vector3(1f / this.moment.x, 1f / this.moment.y, 1f / this.moment.z);
        this.last_postion = this.transform.position;
        this.last_rotation = this.transform.rotation.eulerAngles;
        this.bounding_box = new OctreeItem(this.gameObject);
        this.accumulator = new List<Accumulation>();
    }

    /// <summary>
    /// Update body based on velocity and check if should be awake if asleep.
    /// </summary>
    /// <remarks>Call first for each body.</remarks>
    /// <param name="delta_time"></param>
    public void UpdatePositions(float delta_time)
    {
        if (this.high_mass_collision < 1) this.high_mass_collision = 0;
        if (this.high_mass_collision >= 1) this.high_mass_collision /= 2;
        if (!this.awake)
        {
            // Update BB to current position
            this.bounding_box.center = this.transform.position;
            if (this.velocity.magnitude > 1.0f) this.awake = true;
            else return;
        }

        // Apply velocities
        Vector3 applied_velocity = delta_time * this.velocity;
        Vector3 applied_r_velocity = delta_time * this.r_velocity;
        this.transform.position += applied_velocity;
        this.transform.Rotate((180f / Mathf.PI) * applied_r_velocity, Space.World);

        // Update BB to current position
        this.bounding_box.center = this.transform.position;
    }

    /// <summary>
    /// Get collisions and accumulate resolved delta positions and velocities.
    /// </summary>
    /// <remarks>Call second for each body.</remarks>
    /// <param name="delta_time"></param>
    public void UpdateForCollisions(float delta_time)
    {
        if (!this.awake) return;

        // Check and handle collisions
        foreach (Collision collision in this.GetCollisions(delta_time))
        {
            this.HandleCollision(collision, delta_time);
        }

        // Clear subdivisions
        this.bounding_box.ResetSubdivisions();
    }

    /// <summary>
    /// Apply accumulations and check if body should remain awake.
    /// </summary>
    /// <remarks>Call third and final for each body.</remarks>
    /// <param name="delta_time"></param>
    public void UpdateAccumulations(float delta_time)
    {
        this.ApplyAccumulations();
        if (!this.awake) return;

        if ((this.last_postion - this.transform.position).magnitude <= 0.01f &&
            Body.Vector3DeltaAngle(this.last_rotation, this.transform.rotation.eulerAngles).magnitude <= 0.1f) this.idle_time += delta_time ;
        else this.idle_time = 0;

        if (this.idle_time > 5.0f)
        {
            this.idle_time = 0;
            this.awake = false;
            this.velocity = Vector3.zero;
            this.r_velocity = Vector3.zero;
        }

        this.last_postion = this.transform.position;
        this.last_rotation = this.transform.rotation.eulerAngles;
    }

    public static Vector3 Vector3DeltaAngle(Vector3 a, Vector3 b)
    {
        Vector3 result = new Vector3(
            Mathf.DeltaAngle(a.x, b.x),
            Mathf.DeltaAngle(a.y, b.y),
            Mathf.DeltaAngle(a.z, b.z));
        return result;
    }

    public void ApplyAcceleration(Vector3 acceleration, float delta_time, bool no_wake)
    {
        if (!this.awake && no_wake) return;
        Vector3 applied_acceleration = delta_time * acceleration;
        // TODO: Should use accumulator?
        this.velocity += applied_acceleration;
        //this.AddToAccumulator(Accumulation.Type.velocity, applied_acceleration);
    }

    /// <summary>
    /// Use octree local subdivision to find dynamic collisions.
    /// Use ... to find static collisions.
    /// </summary>
    /// <param name="delta_time"></param>
    /// <returns><see cref="List{Collision}"/> collisions</returns>
    private List<Collision> GetCollisions(float delta_time)
    {
        List<Collision> collisions = new List<Collision>();

        // Get neighbors from subdivisions in octree
        List<Body> neighbors = new List<Body>();
        foreach (Octree subdivision in this.bounding_box.subdivisions[this.bounding_box.subdivisions.Count - 1])
        {
            foreach (OctreeItem neighbor in subdivision.items)
            {
                if (neighbor.body != this) neighbors.Add(neighbor.body);
            }
            // Remove self from subdivision items to prevent rechecking same pairs
            subdivision.items.Remove(this.bounding_box);
        }

        // Find collisions
        collisions.AddRange(Collision.GetCollisionsForDynamicNeighbors(this, neighbors));
        collisions.AddRange(Collision.GetCollisionsForGround(this, delta_time));

        return collisions;
    }

    /// <summary>
    /// Call collision resolution depending on type of collision.
    /// <see cref="Collision.ResolveImpulseDynamicADynamicB(Body, Body, Vector3, Vector3, float)"/>
    /// and ...
    /// </summary>
    /// <param name="collision"></param>
    /// <param name="delta_time"></param>
    private void HandleCollision(Collision collision, float delta_time)
    {
        float depth = collision.depth;
        Body A = this;
        Body B = collision.B;

        // Dyanamic A and Static B
        if (B == null)
        {
            Collision.ResolveImpulseDynamicAGroundB(A, collision.location, collision.normal, depth, delta_time);
        }
        // Dynamic A and B
        else
        {
            Collision.ResolveImpulseDynamicADynamicB(A, B, collision.location, collision.normal, depth, delta_time);
        }
    }

    /// <summary>
    /// Apply accumulations of <see cref="Accumulation.Type"/>.
    /// </summary>
    /// <remarks>Called in <see cref="Update_End(float)"/></remarks>
    private void ApplyAccumulations()
    {
        int count_delta_positions = 0;
        int count_delta_velocity = 0;
        int count_delta_r_velocity = 0;
        Vector3 total_detla_position = Vector3.zero;
        Vector3 total_delta_velocity = Vector3.zero;
        Vector3 total_delta_r_velocity = Vector3.zero;

        // Aggegate accumulations
        foreach (Accumulation a in this.accumulator)
        {
            switch (a.type)
            {
                case Accumulation.Type.position:
                    count_delta_positions += 1;
                    total_detla_position += a.value;
                    break;
                case Accumulation.Type.velocity:
                    count_delta_velocity += 1;
                    total_delta_velocity += a.value;
                    break;
                case Accumulation.Type.r_velocity:
                    count_delta_r_velocity += 1;
                    total_delta_r_velocity += a.value;
                    break;
            }
        }

        // Averages or whatever
        if (count_delta_positions > 0) total_detla_position /= count_delta_positions;
        if (count_delta_velocity > 0) total_delta_velocity /= count_delta_velocity;
        if (count_delta_r_velocity > 0) total_delta_r_velocity /= count_delta_r_velocity;

        this.transform.position += total_detla_position;
        this.bounding_box.center = this.transform.position;
        this.velocity += total_delta_velocity;
        this.r_velocity += total_delta_r_velocity;

        // Clear accumulations
        this.accumulator = new List<Accumulation>();
    }

    /// <summary>
    /// Add accumulation of type <see cref="Accumulation.Type"/>.
    /// </summary>
    /// <remarks>To be applied in <see cref="Update_End(float)"/></remarks>
    /// <param name="type"></param>
    /// <param name="value"></param>
    public void AddToAccumulator(Accumulation.Type type, Vector3 value)
    {
        this.accumulator.Add(new Accumulation(type, value));
    }
}
