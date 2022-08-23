using System.Collections;
using System.Collections.Generic;
using UnityEngine;

public class SimplePhysicConstrainAxis : SimplePhysicConstrain
{
    public Vector3 worldDirection;

    private Vector3 T;
    private Vector3 B;


    SimplePhysicConstrainAxis()
    {
        this.paramSetCount = 2;
    }

    private void OnEnable()
    {
        worldDirection = Vector3.Normalize(worldDirection);
        this.T = Vector3.up;
        if (Vector3.Dot(Vector3.up, worldDirection) > 1e-2)
        {
            this.T = Vector3.right;
        }
        this.B = Vector3.Cross(worldDirection, this.T);
        this.T = Vector3.Cross(this.B, worldDirection);
    }

    public override void AddConstrain(ref int constrainCount, ref float[,] jacobi, ref float[] bias)
    {
        int index = this.rigidbodies[0].rigidIndex;
        if (index < 0) return;
        int start = index * 6;
        jacobi[constrainCount, start    ] = 0;
        jacobi[constrainCount, start + 1] = 0;
        jacobi[constrainCount, start + 2] = 0;
        jacobi[constrainCount, start + 3] = this.T.x;
        jacobi[constrainCount, start + 4] = this.T.y;
        jacobi[constrainCount, start + 5] = this.T.z;
        constrainCount += 1;
        jacobi[constrainCount, start    ] = 0;
        jacobi[constrainCount, start + 1] = 0;
        jacobi[constrainCount, start + 2] = 0;
        jacobi[constrainCount, start + 3] = this.B.x;
        jacobi[constrainCount, start + 4] = this.B.y;
        jacobi[constrainCount, start + 5] = this.B.z;
        constrainCount += 1;
    }
}
