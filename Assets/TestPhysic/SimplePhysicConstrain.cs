using System.Collections;
using System.Collections.Generic;
using UnityEngine;

public abstract class SimplePhysicConstrain
{
    public int paramSetCount = 0;
    public float warmUpLambda = 0;
    public List<SimplePhysicRigidBody> rigidbodies = new List<SimplePhysicRigidBody>();

    public abstract void Setup(Transform transform);

    public abstract void AddConstrain(ref int constrainCount, ref float[,] jacobi, ref float[] bias);

}
