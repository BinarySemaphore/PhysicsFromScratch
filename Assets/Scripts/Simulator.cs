using System.Collections;
using System.Collections.Generic;
using UnityEngine;

public class Simulator : MonoBehaviour
{
    public string objects_tag;

    [HideInInspector]
    public Octree octree_root;
    [HideInInspector]
    public List<OctreeItem> items;

    // Start is called before the first frame update
    void Start()
    {
        this.Initalize();
    }

    private void FixedUpdate()
    {
        this.Initalize();
    }

    void Initalize()
    {
        this.items = new List<OctreeItem>();
        foreach (GameObject game_object in GameObject.FindGameObjectsWithTag(this.objects_tag))
        {
            this.items.Add(new OctreeItem(game_object));
        }
        this.octree_root = new Octree(this.items);
        Octree.Subdivide(this.octree_root);
    }
}
