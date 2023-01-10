using System.Collections;
using System.Collections.Generic;
using UnityEngine;

public class SimplePhysicRigidBodyBox : SimplePhysicRigidBody
{
    private static Vector3[] checkDirs = new Vector3[6]
        {
            new Vector3(1, 0, 0),
            new Vector3(-1, 0, 0),
            new Vector3(0, 1, 0),
            new Vector3(0, -1, 0),
            new Vector3(0, 0, 1),
            new Vector3(0, 0, -1),
        };
    private static Vector3[] checkPos = new Vector3[8]
        {
            new Vector3(1, 1, 1),
            new Vector3(-1, 1, 1),
            new Vector3(1, -1, 1),
            new Vector3(-1, -1, 1),
            new Vector3(1, 1, -1),
            new Vector3(-1, 1, -1),
            new Vector3(1, -1, -1),
            new Vector3(-1, -1, -1),
        };

    public override void InitiateType()
    {
        this.type = RigidbodyType.box;
    }
    public override void CalculateParams()
    {
        Vector3 scale = this.transform.localScale;
        this.mass = scale.x * scale.y * scale.z;
        //this.mass = 1;
        this.inertia = Matrix4x4.identity;
        this.inertia.m00 = this.mass * (scale.y * scale.y + scale.z * scale.z) / 12.0f;
        this.inertia.m11 = this.mass * (scale.x * scale.x + scale.z * scale.z) / 12.0f;
        this.inertia.m22 = this.mass * (scale.x * scale.x + scale.y * scale.y) / 12.0f;
        //this.inertia.m00 = this.mass;
        //this.inertia.m11 = this.mass;
        //this.inertia.m22 = this.mass;
    }

    public override Vector3 SupportFunction(ref Vector3 worldDir)
    {
        var dir = Vector3.Normalize(this.WorldTDiroLocal(worldDir));
        Vector3 faceDir = Vector3.zero;
        float faceDot = 0;
        for (int i = 0; i < checkDirs.Length; ++i)
        {
            var face = checkDirs[i];
            var dot = Vector3.Dot(face, dir);
            if(dot > faceDot)
            {
                faceDot = dot;
                faceDir = face;
            }
        }

        if (faceDot > 1 - SimplePhysicSolver._Epsilon)
        {
            worldDir = this.unsolvedRotation.MultiplyPoint3x4(faceDir);
            return this.LocalDirToWorld(Vector3.Scale(faceDir, this.transform.localScale * 0.5f));
        }

        var res = faceDir; 
        for(int i = 0; i < 3; ++i)
        {
            if(faceDir[i] == 0 && Mathf.Abs(dir[i]) <= SimplePhysicSolver._Epsilon)
            {
                for (int j = 0; j < 3; ++j)
                {
                    if (faceDir[j] == 0 && i != j)
                    {
                        res[j] = Mathf.Sign(dir[j]);
                        return this.LocalDirToWorld(Vector3.Scale(res, this.transform.localScale * 0.5f));
                    }
                }
            }
        }

        float resDot = 0;
        for (int i = 0; i < checkPos.Length; ++i)
        {
            var pos = checkPos[i];
            var dot = Vector3.Dot(pos, dir);
            if (dot > resDot)
            {
                resDot = dot;
                res = pos;
            }
        }
        return this.LocalDirToWorld(Vector3.Scale(res, this.transform.localScale * 0.5f));
    }
}
