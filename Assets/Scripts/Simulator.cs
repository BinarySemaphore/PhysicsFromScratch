using System.Collections;
using System.Collections.Generic;
using UnityEngine;

public class Simulator : MonoBehaviour
{
    public List<GameObject> to_simulate;

    [HideInInspector]
    public Octree octree_root;
    [HideInInspector]
    public List<OctreeItem> items;

    // Start is called before the first frame update
    void Start()
    {
        this.items = new List<OctreeItem>();
        foreach (GameObject game_object in to_simulate)
        {
            this.items.Add(new OctreeItem(game_object));
        }
        this.octree_root = new Octree(this.items);
        Octree.Subdivide(this.octree_root);
    }

    // Update is called once per frame
    void Update()
    {
        
    }
}
