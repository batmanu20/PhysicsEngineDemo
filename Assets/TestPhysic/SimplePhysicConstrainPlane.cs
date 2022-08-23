using System.Collections;
using System.Collections.Generic;
using UnityEngine;

public class SimplePhysicConstrainPlane : SimplePhysicConstrain
{
    public Vector3 normal;

    SimplePhysicConstrainPlane()
    {
        this.paramSetCount = 1;
    }

    public override void AddConstrain(ref int constrainCount, ref float[,] jacobi, ref float[] bias)
    {
        int index = this.rigidbodies[0].rigidIndex;
        if (index < 0) return;
        int start = index * 6;
        // 2 *P0 - x0, 2 * P1 - y0 ...
        jacobi[constrainCount, start    ] = normal.x;
        jacobi[constrainCount, start + 1] = normal.y;
        jacobi[constrainCount, start + 2] = normal.z;
        jacobi[constrainCount, start + 3] = 0;
        jacobi[constrainCount, start + 4] = 0;
        jacobi[constrainCount, start + 5] = 0;
        constrainCount += 1;
    }
}
