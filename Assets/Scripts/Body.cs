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
public class Body : OctreeItem
{
    

    public bool awake;
    public int idle_count;
    public float mass;
    public float friction;
    public float elasticity;
    public Vector3 velocity;
    public Vector3 r_velocity;
    public Vector3 last_postion;
    public Quaternion last_rotation;
    public GameObject entity;
    public List<Accumulation> accumulator;

    /// <summary>
    /// Create <see cref="Body"/> from <paramref name="entity"/>.
    /// </summary>
    /// <param name="entity"></param>
    public Body(GameObject entity) : base(entity)
    {
        this.awake = true;
        this.idle_count = 0;
        this.mass = 1.0f;
        this.friction = 0.5f;
        this.elasticity = 0.5f;
        this.velocity = Vector3.zero;
        this.r_velocity = Vector3.zero;
        this.last_postion = this.center;
        this.last_rotation = entity.transform.rotation;
        this.entity = entity;
        this.accumulator = new List<Accumulation>();
    }

    /// <summary>
    /// Call first once per simulation update.
    /// Follow up updates with <see cref="Update_End"/>
    /// </summary>
    /// <param name="delta_time"></param>
    public void Update(float delta_time)
    {
        if (!this.awake)
        {
            if (this.velocity.magnitude > 0.1f) this.awake = true;
            else return;
        }

        // Apply velocities
        Vector3 applied_velocity = delta_time * this.velocity;
        Vector3 applied_r_velocity = delta_time * this.r_velocity;
        this.entity.transform.position += applied_velocity;
        this.entity.transform.rotation = Quaternion.Euler(applied_r_velocity) * this.entity.transform.rotation;

        // Update BB to current position
        this.center = this.entity.transform.position;

        // Check and handle collisions
        foreach (Collision collision in this.GetCollisions(delta_time))
        {
            this.HandleCollision(collision, delta_time);
        }

        // Clear subdivisions
        this.ResetSubdivisions();
    }

    /// <summary>
    /// Call once per simulation update after <see cref="Update(float)"/>
    /// </summary>
    public void Update_End(float delta_time)
    {
        if (!this.awake) return;

        this.ApplyAccumulations();

        if ((this.last_postion - this.center).magnitude <= 0.001f) this.idle_count += 1;
        else this.idle_count = 0;

        if (this.idle_count > 10)
        {
            this.idle_count = 0;
            this.awake = false;
            this.velocity = Vector3.zero;
            this.r_velocity = Vector3.zero;
            this.accumulator = new List<Accumulation>();
        }
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
        foreach (Octree subdivision in this.subdivisions[-1])
        {
            foreach (Body neighbor in subdivision.items)
            {
                if (neighbor != this) neighbors.Add(neighbor);
            }
            // Remove self from subdivision items to prevent rechecking same pairs
            subdivision.items.Remove(this);
        }

        // Find collisions
        collisions.AddRange(Collision.GetCollisionsForDynamicNeighbors(this, neighbors));
        // TODO: Terrain/non-dynamic collisions

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
            // TODO: ResolveImpulseDynamicAStaticB
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
                    total_delta_velocity += a.value;
                    break;
                case Accumulation.Type.r_velocity:
                    total_delta_r_velocity += a.value;
                    break;
            }
        }

        // Averages or whatever
        total_detla_position = total_detla_position / count_delta_positions;

        this.entity.transform.position += total_detla_position;
        this.center = this.entity.transform.position;
        this.velocity += total_delta_velocity;
        this.r_velocity += total_delta_r_velocity;
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
