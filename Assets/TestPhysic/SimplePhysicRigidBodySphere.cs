using System.Collections;
using System.Collections.Generic;
using UnityEngine;

public class SimplePhysicRigidBodySphere : SimplePhysicRigidBody
{
    public override void InitiateType()
    {
        this.type = RigidbodyType.sphere;
    }
    public override void CalculateParams()
    {
        Vector3 scale = this.transform.localScale;
        this.mass = scale.x * scale.y * scale.z * 1.333f * Mathf.PI / 8.0f;
        //this.mass = 1;
        this.inertia = Matrix4x4.identity;
        this.inertia.m00 = this.mass * (scale.y * scale.y + scale.z * scale.z) * 0.4f / (4.0f * 12.0f);
        this.inertia.m11 = this.mass * (scale.x * scale.x + scale.z * scale.z) * 0.4f / (4.0f * 12.0f);
        this.inertia.m22 = this.mass * (scale.x * scale.x + scale.y * scale.y) * 0.4f / (4.0f * 12.0f);
    }

    public override Vector3 SupportFunction(ref Vector3 worldDir)
    {
        var dir = Vector3.Normalize(this.WorldTDiroLocal(worldDir));
        return this.LocalDirToWorld(Vector3.Scale(dir, this.transform.localScale * 0.5f));
    }
}
