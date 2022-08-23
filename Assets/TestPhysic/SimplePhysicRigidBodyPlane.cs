using System.Collections;
using System.Collections.Generic;
using UnityEngine;

public class SimplePhysicRigidBodyPlane : SimplePhysicRigidBody
{
    public override void InitiateType()
    {
        this.type = RigidbodyType.upFace;
    }
    public override void CalculateParams()
    {
        Vector3 scale = this.transform.localScale;
        this.mass = scale.x * scale.z / 1000;
        this.mass = 1;
        this.inertia = Matrix4x4.identity;
        this.inertia.m00 = this.mass * (1 + scale.z * scale.z) / 12.0f;
        this.inertia.m11 = this.mass * (scale.x * scale.x + scale.z * scale.z) / 12.0f;
        this.inertia.m22 = this.mass * (scale.x * scale.x + 1) / 12.0f;
    }

    public override Vector3 SupportFunction(ref Vector3 worldDir)
    {
        var dir = Vector3.Normalize(this.WorldTDiroLocal(worldDir));
        var dot = Vector3.Dot(dir, this.transform.up);
        if (dot > 1 - SimplePhysicSolver._Epsilon)
        {
            worldDir = this.unsolvedRotation.MultiplyPoint3x4(this.transform.up);
            return this.transform.position;
        }

        dir.y = 0;
        dir = Vector3.Normalize(dir);
        dir.Scale(this.transform.localScale * 5.0f);
        return this.LocalDirToWorld(dir);
    }
}
