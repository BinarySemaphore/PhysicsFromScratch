using System.Collections;
using System.Collections.Generic;
using UnityEngine;

public class Simulator : MonoBehaviour
{
    public float simulation_speed;
    public string objects_tag;
    public string grounds_tag;
    public Vector3 gravity;

    [HideInInspector]
    public Octree octree_root;
    [HideInInspector]
    public List<Body> bodies;

    // Start is called before the first frame update
    void Start()
    {
        this.Initalize();
    }

    private void FixedUpdate()
    {
        this.UpdateSim(Time.deltaTime * this.simulation_speed); // Time.deltaTime | 0.01f
        // TODO: Check if number of GameObjects with tag changed and update this.bodies
    }

    private void Initalize()
    {
        this.bodies = new List<Body>();
        foreach (GameObject game_object in GameObject.FindGameObjectsWithTag(this.objects_tag))
        {
            AABB_Object info = game_object.GetComponent<AABB_Object>();
            Body new_body = new Body(game_object);
            new_body.mass = info.mass;
            new_body.friction = info.friction;
            new_body.elasticity = info.elasticity;
            new_body.velocity = info.starting_velocity;
            new_body.sim = this;
            this.bodies.Add(new_body);
        }
        this.octree_root = new Octree(this.bodies);
        Octree.Subdivide(this.octree_root);
    }

    private void UpdateSim(float delta_time)
    {
        this.octree_root = new Octree(this.bodies);
        Octree.Subdivide(this.octree_root);
        foreach (Body body in this.bodies)
        {
            if (body.entity.GetComponent<AABB_Object>().apply_gravity)
            {
                body.ApplyAcceleration(this.gravity, delta_time, true);
            }
            body.Update(delta_time);
        }
        foreach (Body body in this.bodies)
        {
            body.Update_End(delta_time);
        }
    }
}
