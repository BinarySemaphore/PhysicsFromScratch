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
    public List<AABB_Object> objects_to_draw;

    // Start is called before the first frame update
    void Start()
    {
        cache_index = 0;
        cache_objects = new List<GameObject>();
    }

    // Update is called once per frame
    void FixedUpdate()
    {
        foreach (AABB_Object item in objects_to_draw)
        {
            foreach (Vector3 point in item.bounding_box.GetDrawPoints())
            {
                DrawCircle(point, 0.075f, width:0.1f);
            }
        }
        CleanCache();
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
        LineRenderer renderer = GetRenderer();
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

    /// <summary>
    /// Call at end of update method to clean cache.
    /// Culls any drawing object not used.
    /// </summary>
    void CleanCache()
    {
        for (int i = cache_index; i < cache_objects.Count; i++)
        {
            GameObject.Destroy(cache_objects[cache_index]);
            cache_objects.RemoveAt(cache_index);
        }
        cache_index = 0;
    }

    /// <summary>
    /// Get existing or new rendering object, cache if new.
    /// Requires <see cref="CleanCache"/>.
    /// </summary>
    /// <returns></returns>
    LineRenderer GetRenderer()
    {
        LineRenderer renderer = null;
        if (cache_index == cache_objects.Count)
        {
            GameObject new_object = new GameObject();
            new_object.transform.parent = this.transform;
            renderer = new_object.AddComponent<LineRenderer>();
            renderer.receiveShadows = false;
            renderer.shadowCastingMode = UnityEngine.Rendering.ShadowCastingMode.Off;
            cache_objects.Add(new_object);
        }
        if (renderer == null)
        {
            renderer = cache_objects[cache_index].GetComponent<LineRenderer>();
        }
        cache_index++;
        return renderer;
    }
}
