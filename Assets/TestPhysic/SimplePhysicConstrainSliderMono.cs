using System.Collections;
using System.Collections.Generic;
using UnityEngine;

public class SimplePhysicConstrainSliderMono : MonoBehaviour
{
    public Vector3 worldDirection;

    public SimplePhysicConstrainSlider slider = new SimplePhysicConstrainSlider();
    private void OnEnable()
    {
        slider.worldDirection = this.worldDirection;
        slider.Setup(transform);
        var rigid = this.GetComponent<SimplePhysicRigidBody>();
        if (rigid != null)
        {
            slider.rigidbodies.Add(rigid);
            rigid.staticConstrains.Add(slider);
        }
    }
}

public class SimplePhysicConstrainSlider : SimplePhysicConstrain
{
    public Vector3 worldDirection;
    private Vector3 T;
    private Vector3 B;

    public SimplePhysicConstrainSlider()
    {
        this.paramSetCount = 2;
    }

    public override void Setup(Transform transform)
    {
        worldDirection = Vector3.Normalize(worldDirection);
        this.T = Vector3.up;
        if (Vector3.Dot(Vector3.up, worldDirection) > 1 - 1e-2)
        {
            this.T = Vector3.right;
        }
        this.B = Vector3.Cross(worldDirection, this.T);
        this.T = Vector3.Cross(this.B, worldDirection);
    }

    private void OnEnable()
    {
        worldDirection = Vector3.Normalize(worldDirection);
        this.T = Vector3.up;
        if (Vector3.Dot(Vector3.up, worldDirection) > 1 - 1e-2)
        {
            this.T = Vector3.right;
        }
        this.B = Vector3.Cross(worldDirection, this.T);
        this.T = Vector3.Cross(this.B, worldDirection);
    }

    public override void AddConstrain(ref int constrainCount, ref float[,] jacobi, ref float[] bias)
    {
        if (this.rigidbodies.Count != 1) return;
        int index = this.rigidbodies[0].rigidIndex;
        if (index < 0) return;
        int start = index * 6;

        jacobi[constrainCount, start    ] = this.T.x;
        jacobi[constrainCount, start + 1] = this.T.y;
        jacobi[constrainCount, start + 2] = this.T.z;
        jacobi[constrainCount, start + 3] = 0;
        jacobi[constrainCount, start + 4] = 0;
        jacobi[constrainCount, start + 5] = 0;
        constrainCount += 1;
        jacobi[constrainCount, start    ] = this.B.x;
        jacobi[constrainCount, start + 1] = this.B.y;
        jacobi[constrainCount, start + 2] = this.B.z;
        jacobi[constrainCount, start + 3] = 0;
        jacobi[constrainCount, start + 4] = 0;
        jacobi[constrainCount, start + 5] = 0;
        constrainCount += 1;
    }
}