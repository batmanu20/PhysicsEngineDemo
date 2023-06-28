using System.Collections;
using System.Collections.Generic;
using UnityEngine;

public class SimplePhysicConstrainCollision : SimplePhysicConstrain
{
    public int idA;
    public int idB;
    public SimplePhysicRigidBody.CollisionResult collision;
    public float lambdaN;

    private int contactFrames = 0;
    private float[,] p = new float[3,12];
    private float[] b = new float[3];
    private Queue<SimplePhysicRigidBody.CollisionResult> manifold = new Queue<SimplePhysicRigidBody.CollisionResult>();

    private const int warmupPoints = 4;
    public SimplePhysicConstrainCollision(SimplePhysicRigidBody.CollisionResult _collision)
    {
        collision = _collision;
    }

    public void AddConstrain(SimplePhysicRigidBody.CollisionResult _collision)
    {
        manifold.Enqueue(collision);
        if(manifold.Count > warmupPoints)
        {
            manifold.Dequeue();
        }
        collision = _collision;
    }

    public void SetParameter(float[] _p, float _b,int _idA, int _idB, int paramSet)
    {
        for(int i = 0;i < _p.Length; ++i)
        {
            p[paramSet, i] = _p[i];
        }
        b[paramSet] = _b;
        idA = _idA;
        idB = _idB;
        contactFrames++;
    }

    public override void AddConstrain(ref int constrainCount, ref float[,] jacobi, ref float[] bias)
    {
        for (int set = 0; set < 3; ++set)
        {
            int start = idA * 6;
            for (int i = 0; i < 6; ++i)
            {
                jacobi[constrainCount, start + i] = p[set, i];
            }
            start = idB * 6;
            for (int i = 0; i < 6; ++i)
            {
                jacobi[constrainCount, start + i] = p[set, i + 6];
            }
            bias[constrainCount] = b[set];
            constrainCount += 1;
        }
    }

    public void Warmup(SimplePhysicSolver solver)
    {
        if(manifold.Count >= warmupPoints + 1)
        {
            for(int i = 0; i < warmupPoints; i++)
            {
                var c = manifold.ToArray()[i];
                //var impulse =
            }
        }
    }

    public override void Setup(Transform transform)
    {
        throw new System.NotImplementedException();
    }
}
