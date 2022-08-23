using System.Collections;
using System.Collections.Generic;
using UnityEngine;

public abstract class SimplePhysicConstrain : MonoBehaviour
{
    public enum ConstrainType
    {
        NonContact,
        ContactNormal,
        ContactTangent,
    }
    public int paramSetCount = 0;
    public float warmUpLambda = 0;
    public List<SimplePhysicRigidBody> rigidbodies = new List<SimplePhysicRigidBody>();
    public ConstrainType type = ConstrainType.NonContact;

    public abstract void AddConstrain(ref int constrainCount, ref float[,] jacobi, ref float[] bias);

}
