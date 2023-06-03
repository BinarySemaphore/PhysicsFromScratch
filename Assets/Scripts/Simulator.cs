using System.Collections;
using System.Collections.Generic;
using UnityEngine;

public class Simulator : MonoBehaviour
{
    public string objects_tag;

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
        this.UpdateSim(0.001f);
        // TODO: Check if number of GameObjects with tag changed and update this.bodies
    }

    private void Initalize()
    {
        this.bodies = new List<Body>();
        foreach (GameObject game_object in GameObject.FindGameObjectsWithTag(this.objects_tag))
        {
            AABB_Object info = game_object.GetComponent<AABB_Object>();
            Body new_body = new Body(game_object);
            this.bodies.Add(new_body);
            new_body.mass = info.mass;
            new_body.friction = info.friction;
            new_body.elasticity = info.elasticity;
            new_body.velocity = info.starting_velocity;
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
            body.Update(delta_time);
        }
        foreach (Body body in this.bodies)
        {
            body.Update_End(delta_time);
        }
    }
}
