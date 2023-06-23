using System.Collections;
using System.Collections.Generic;
using UnityEngine;

public class Simulator : MonoBehaviour
{
    private bool initialized;

    public int iterations;
    public float max_delta_time;
    public float simulation_speed;
    public string objects_tag;
    public string grounds_tag;
    public Vector3 gravity;

    [HideInInspector]
    public Octree octree_root;
    [HideInInspector]
    public List<OctreeItem> items;
    [HideInInspector]
    public List<Body> bodies;

    // Start is called before the first frame update
    void Start()
    {
        this.initialized = false;
    }

    private void FixedUpdate()
    {
        if (!this.initialized)
        {
            this.Initalize();
            this.initialized = true;
        }
        float delta_time = Time.fixedDeltaTime;
        delta_time = Mathf.Clamp(delta_time, 0.000001f, this.max_delta_time);
        this.UpdateSim(delta_time * this.simulation_speed, iterations);
        // TODO: Check if number of GameObjects with tag changed and update this.bodies
    }

    private void Initalize()
    {
        this.items = new List<OctreeItem>();
        this.bodies = new List<Body>();
        foreach (GameObject game_object in GameObject.FindGameObjectsWithTag(this.objects_tag))
        {
            Body body = game_object.GetComponent<Body>();
            body.sim = this;
            this.bodies.Add(body);
            this.items.Add(body.bounding_box);
        }
        this.octree_root = new Octree(this.items);
        Octree.Subdivide(this.octree_root);
    }

    private void UpdateSim(float delta_time, int iterations)
    {
        float iterative_delta_time = delta_time / (float)iterations;
        for (int i = 0; i < iterations; i++)
        {
            this.UpdateIteration(iterative_delta_time);
        }
    }

    private void UpdateIteration(float delta_time)
    {
        this.octree_root = new Octree(this.items);
        Octree.Subdivide(this.octree_root);
        foreach (Body body in this.bodies)
        {
            if (body.apply_gravity)
            {
                body.ApplyAcceleration(this.gravity, delta_time, true);
            }
            body.UpdatePositions(delta_time);
        }
        foreach (Body body in this.bodies)
        {
            body.UpdateForCollisions(delta_time);
        }
        foreach (Body body in this.bodies)
        {
            body.UpdateAccumulations(delta_time);
        }
    }
}
