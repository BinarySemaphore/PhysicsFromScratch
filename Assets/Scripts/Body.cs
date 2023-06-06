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
    public bool apply_gravity;
    public bool awake;
    public float idle_time;
    public float mass;
    public float friction;
    public float elasticity;
    public Vector3 velocity;
    public Vector3 r_velocity;
    public Vector3 last_postion;
    public Quaternion last_rotation;
    public OctreeItem bounding_box;
    public List<Accumulation> accumulator;
    public Simulator sim;

    // Start is called before the first frame update
    void Start()
    {
        this.awake = true;
        this.idle_time = 0;
        this.last_postion = this.transform.position;
        this.last_rotation = this.transform.rotation;
        this.bounding_box = new OctreeItem(this.gameObject);
        this.accumulator = new List<Accumulation>();
    }

    /// <summary>
    /// Call first once per simulation update.
    /// Follow up updates with <see cref="Update_End"/>
    /// </summary>
    /// <param name="delta_time"></param>
    public void UpdateBody(float delta_time)
    {

        if (!this.awake)
        {
            // Update BB to current position
            this.bounding_box.center = this.transform.position;
            if (this.velocity.magnitude > 0.5f) this.awake = true;
            else return;
        }

        // Apply velocities
        Vector3 applied_velocity = delta_time * this.velocity;
        Vector3 applied_r_velocity = delta_time * this.r_velocity;
        this.transform.position += applied_velocity;
        this.transform.rotation = Quaternion.Euler(applied_r_velocity) * this.transform.rotation;

        // Update BB to current position
        this.bounding_box.center = this.transform.position;

        // Check and handle collisions
        foreach (Collision collision in this.GetCollisions(delta_time))
        {
            this.HandleCollision(collision, delta_time);
        }

        // Clear subdivisions
        this.bounding_box.ResetSubdivisions();
    }

    /// <summary>
    /// Call once per simulation update after <see cref="Update(float)"/>
    /// </summary>
    public void UpdateEnd(float delta_time)
    {
        this.ApplyAccumulations();
        if (!this.awake) return;

        if ((this.last_postion - this.transform.position).magnitude <= 0.1f && this.velocity.magnitude <= 3.0f) this.idle_time += delta_time ;
        else this.idle_time = 0;

        if (this.idle_time > 2.0f)
        {
            this.idle_time = 0;
            this.awake = false;
            this.velocity = Vector3.zero;
            this.r_velocity = Vector3.zero;
        }

        this.last_postion = this.transform.position;
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
        Body A = this;
        Body B = collision.B;

        // Dyanamic A and Static B
        if (B == null)
        {
            Collision.ResolveImpulseDynamicAGroundB(A, collision.location, collision.normal, delta_time);
        }
        // Dynamic A and B
        else
        {
            Collision.ResolveImpulseDynamicADynamicB(A, B, collision.location, collision.normal, delta_time);
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
                    total_delta_r_velocity += a.value;
                    break;
            }
        }

        // Averages or whatever
        if (count_delta_positions > 0) total_detla_position /= count_delta_positions;
        if (count_delta_velocity > 0) total_delta_velocity /= count_delta_velocity;

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
