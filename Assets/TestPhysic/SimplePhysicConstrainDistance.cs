using System.Collections;
using System.Collections.Generic;
using UnityEngine;

public class SimplePhysicConstrainDistance : SimplePhysicConstrain
{
    public Transform pinPoint;

    private Vector3 worldPosition;

    SimplePhysicConstrainDistance()
    {
        this.paramSetCount = 1;
    }

    public override void Setup(Transform transform)
    {
        if (pinPoint != null)
        {
            worldPosition = pinPoint.position;
        }
        else
        {
            worldPosition = transform.position;
        }
    }

    public override void AddConstrain(ref int constrainCount, ref float[,] jacobi, ref float[] bias)
    {
        int index = this.rigidbodies[0].rigidIndex;
        if (index < 0) return;
        int start = index * 6;
        jacobi[constrainCount, start] = this.rigidbodies[0].transform.position.x - worldPosition.x;
        jacobi[constrainCount, start + 1] = this.rigidbodies[0].transform.position.y - worldPosition.y;
        jacobi[constrainCount, start + 2] = this.rigidbodies[0].transform.position.z - worldPosition.z;
        jacobi[constrainCount, start + 3] = 0;
        jacobi[constrainCount, start + 4] = 0;
        jacobi[constrainCount, start + 5] = 0;
        constrainCount += 1;
    }
}
