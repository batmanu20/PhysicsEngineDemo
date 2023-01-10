using System.Collections;
using System.Collections.Generic;
using UnityEngine;

public class SimplePhysicConstrainJoint : SimplePhysicConstrain
{
    public Transform pinPoint;

    private Vector3 worldPosition;
    private Vector3 worldDirection;

    SimplePhysicConstrainJoint()
    {
        this.paramSetCount = 2;
    }

    public override void Setup(Transform transform)
    {
        if (pinPoint != null)
        {
            worldPosition = pinPoint.position;
            //worldDirection = this.pinPoint.position - this.transform.position;
            //worldDirection = Vector3.Normalize(worldDirection);
        }
        else
        {
            worldPosition = transform.position;
            //worldDirection = Vector3.Normalize(this.transform.forward);
        }
    }

    private void OnEnable()
    {
        //if (pinPoint != null)
        //{
        //    worldPosition = pinPoint.position;
        //    //worldDirection = this.pinPoint.position - this.transform.position;
        //    //worldDirection = Vector3.Normalize(worldDirection);
        //}
        //else
        //{
        //    worldPosition = this.transform.position;
        //    //worldDirection = Vector3.Normalize(this.transform.forward);
        //}
    }

    public override void AddConstrain(ref int constrainCount, ref float[,] jacobi, ref float[] bias)
    {
        int index = this.rigidbodies[0].rigidIndex;
        if (index < 0) return;
        int start = index * 6;
        worldDirection = Vector3.Normalize(this.pinPoint.position - this.rigidbodies[0].transform.position);
        jacobi[constrainCount, start    ] = this.rigidbodies[0].transform.position.x - worldPosition.x;
        jacobi[constrainCount, start + 1] = this.rigidbodies[0].transform.position.y - worldPosition.y;
        jacobi[constrainCount, start + 2] = this.rigidbodies[0].transform.position.z - worldPosition.z;
        jacobi[constrainCount, start + 3] = 0;
        jacobi[constrainCount, start + 4] = 0;
        jacobi[constrainCount, start + 5] = 0;
        constrainCount += 1;
        jacobi[constrainCount, start    ] = 0;
        jacobi[constrainCount, start + 1] = 0;
        jacobi[constrainCount, start + 2] = 0;
        jacobi[constrainCount, start + 3] = this.rigidbodies[0].transform.position.x - worldPosition.x;
        jacobi[constrainCount, start + 4] = this.rigidbodies[0].transform.position.y - worldPosition.y;
        jacobi[constrainCount, start + 5] = this.rigidbodies[0].transform.position.z - worldPosition.z;
        constrainCount += 1;
    }
}
