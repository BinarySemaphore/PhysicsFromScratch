using System.Collections;
using System.Collections.Generic;
using UnityEngine;

/// <summary>
/// Attachment of <see cref="AABB"/> to <see cref="GameObject"/>.
/// </summary>
public class AABB_Object : MonoBehaviour
{
    public bool apply_gravity;
    public float mass;
    public float friction;
    public float elasticity;
    public Vector3 starting_velocity;
    // Start is called before the first frame update
    void Start()
    {
    }

    private void FixedUpdate()
    {
    }
}
