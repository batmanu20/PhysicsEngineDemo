using System.Collections;
using System.Collections.Generic;
using UnityEngine;

public class SimplePhysicConstrainCollision : SimplePhysicConstrain
{
    private float[] p;
    private float b;
    private int idA;
    private int idB;

    public SimplePhysicConstrainCollision(float[] _p, float _b,int _idA, int _idB, bool normal)
    {
        p = _p;
        b = _b;
        idA = _idA;
        idB = _idB;
        this.type = normal ? ConstrainType.ContactNormal : ConstrainType.ContactTangent;
    }

    public override void AddConstrain(ref int constrainCount, ref float[,] jacobi, ref float[] bias)
    {
        int start = idA * 6;
        for(int i = 0; i < 6; ++i)
        {
            jacobi[constrainCount, start + i] = p[i];
        }
        start = idB * 6;
        for (int i = 0; i < 6; ++i)
        {
            jacobi[constrainCount, start + i] = p[i + 6];
        }
        bias[constrainCount] = b;
        constrainCount += 1;
    }
}
