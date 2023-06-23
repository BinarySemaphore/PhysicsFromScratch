using System.Collections;
using System.Collections.Generic;
using UnityEngine;

/// <summary>
/// Will draw circles around points of <see cref="AABB"/> from given list of <see cref="objects_to_draw"/>.
/// </summary>
public class Drawer : MonoBehaviour
{
    private int cache_index;
    private List<GameObject> cache_objects;

    public new Camera camera;
    public List<GameObject> boxes;
    public Simulator sim;

    // Start is called before the first frame update
    void Start()
    {
        this.cache_index = 0;
        this.cache_objects = new List<GameObject>();
    }

    // Update is called once per frame
    void FixedUpdate()
    {
        if (this.sim != null)
        {
            this.DrawOctree(this.sim.octree_root);
            foreach (OctreeItem item in this.sim.items)
            {
                this.DrawAABB(item);
            }
            foreach (Body body in this.sim.bodies)
            {
                foreach (Collision collision in body.collisions_encountered)
                {
                    this.DrawCollision(collision, width: 0.1f);
                }
            }
        }
        else
        {
            List<OBB> obbs = new List<OBB>();
            foreach (GameObject box in this.boxes)
            {
                OBB bb = new OBB(box.transform.position, box.transform.rotation, box.transform.localScale);
                this.DrawBox(bb.GetVerticies(), width: 0.1f);
                obbs.Add(bb);
            }

            Vector3 position;
            Vector3 normal;
            float depth;
            for (int i = 0; i < obbs.Count - 1; i++)
            {
                for (int j = i + 1; j < obbs.Count; j++)
                {
                    if (OBB.IsOverlapping(obbs[i], obbs[j], out position, out normal, out depth))
                    {
                        this.DrawCircle(position, 0.75f, width: 0.1f);
                        this.DrawLine(position, position + normal, width: 0.1f);
                        Debug.Log($"Depth: {depth}");
                    }
                }
            }
        }

        this.CleanCache();
    }

    /// <summary>
    /// Draw <see cref="AABB"/> corners.
    /// </summary>
    /// <param name="bounding_box"></param>
    public void DrawAABB(AABB bounding_box)
    {
        /*
        foreach (Vector3 point in bounding_box.GetDrawPoints())
        {
            this.DrawCircle(point, 0.075f, width: 0.1f);
        }*/
        this.DrawBox(bounding_box.GetDrawPoints(), width: 0.1f);
    }

    /// <summary>
    /// Draw <see cref="Octree"/> corners and its children.
    /// </summary>
    /// <param name="current"></param>
    public void DrawOctree(Octree current)
    {
        if (current == null) return;

        this.DrawAABB(current);
        foreach (Octree child in current.children)
        {
            this.DrawOctree(child);
        }
    }

    void DrawLine(Vector3 start, Vector3 end, float width = 1.0f)
    {
        LineRenderer renderer = this.GetRenderer();
        renderer.positionCount = 2;
        renderer.startWidth = width;
        renderer.endWidth = width;
        renderer.SetPosition(0, start);
        renderer.SetPosition(1, end);
    }

    /// <summary>
    /// Draw a circle at <paramref name="center"/> with <paramref name="radius"/>.
    /// </summary>
    /// <param name="center"></param>
    /// <param name="radius"></param>
    /// <param name="steps">Number of lines to use for circle.</param>
    /// <param name="width">Line width.</param>
    void DrawCircle(Vector3 center, float radius, int steps = 20, float width = 1.0f)
    {
        LineRenderer renderer = this.GetRenderer();
        float angle_step = 2 * Mathf.PI / steps;
        renderer.positionCount = steps + 2;
        renderer.startWidth = width;
        renderer.endWidth = width;
        for (int i = 0; i < steps + 2; i++)
        {
            float x = radius * Mathf.Cos(i * angle_step);
            float y = radius * Mathf.Sin(i * angle_step);
            renderer.SetPosition(i, new Vector3(x, y, 0) + center);
        }
    }

    void DrawBox(List<Vector3> corners, float width = 1.0f)
    {
        LineRenderer renderer = this.GetRenderer();
        renderer.positionCount = 16;
        renderer.startWidth = width;
        renderer.endWidth = width;

        int[] indices = { 1, 0, 2, 3, 1, 5, 4, 6, 7, 5, 4, 0, 2, 6, 7, 3 };
        for (int i = 0; i < indices.Length; i++)
        {
            renderer.SetPosition(i, corners[indices[i]]);
        }
    }

    void DrawCollision(Collision collision, float width = 1.0f)
    {
        Vector3 position_to_A = collision.A.transform.position - collision.location;
        Vector3 velocity_A_from_rotation = Vector3.Cross(position_to_A, collision.A.r_velocity);
        Vector3 velocity_A_at_position = collision.A.velocity + velocity_A_from_rotation;
        float acting_velocity_A = Vector3.Dot(velocity_A_at_position, collision.normal);
        float relative_velocity = acting_velocity_A;

        if (collision.B != null)
        {
            Vector3 position_to_B = collision.B.transform.position - collision.location;
            Vector3 velocity_B_from_rotation = Vector3.Cross(position_to_B, collision.B.r_velocity);
            Vector3 velocity_B_at_position = collision.B.velocity + velocity_B_from_rotation;
            float acting_velocity_B = Vector3.Dot(velocity_B_at_position, collision.normal);
            relative_velocity -= acting_velocity_B;
        }

        this.DrawBox(collision.A.GetPreciseCollider().GetVerticies(), width: width);
        this.DrawCircle(collision.location, 0.5f, width: width);
        this.DrawLine(collision.location, collision.location + collision.normal, width: width);
        this.DrawLine(collision.location, collision.location + relative_velocity * Vector3.up, width: width);
    }

    /// <summary>
    /// Call at end of update method to clean cache.
    /// Culls any drawing object not used.
    /// </summary>
    void CleanCache()
    {
        for (int i = this.cache_index; i < this.cache_objects.Count; i++)
        {
            GameObject.Destroy(this.cache_objects[this.cache_index]);
            this.cache_objects.RemoveAt(this.cache_index);
        }
        this.cache_index = 0;
    }

    /// <summary>
    /// Get existing or new rendering object, cache if new.
    /// Requires <see cref="CleanCache"/>.
    /// </summary>
    /// <returns></returns>
    LineRenderer GetRenderer()
    {
        LineRenderer renderer = null;
        if (this.cache_index == this.cache_objects.Count)
        {
            GameObject new_object = new GameObject();
            new_object.transform.parent = this.transform;
            renderer = new_object.AddComponent<LineRenderer>();
            renderer.receiveShadows = false;
            renderer.shadowCastingMode = UnityEngine.Rendering.ShadowCastingMode.Off;
            this.cache_objects.Add(new_object);
        }
        if (renderer == null)
        {
            renderer = this.cache_objects[this.cache_index].GetComponent<LineRenderer>();
        }
        this.cache_index++;
        return renderer;
    }
}
