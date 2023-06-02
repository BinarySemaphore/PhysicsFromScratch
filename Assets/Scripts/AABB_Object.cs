using System.Collections;
using System.Collections.Generic;
using UnityEngine;

/// <summary>
/// Attachment of <see cref="AABB"/> to <see cref="GameObject"/>.
/// </summary>
public class AABB_Object : MonoBehaviour
{
    public AABB bounding_box;
    // Start is called before the first frame update
    void Start()
    {
        bounding_box = new AABB(this.gameObject, AABB.Containment.smallest);
    }

    private void FixedUpdate()
    {
        bounding_box = new AABB(this.gameObject, AABB.Containment.smallest);
    }
}
