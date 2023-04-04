using System.Collections;
using System.Collections.Generic;
using UnityEngine;

public class SimplePhysicConstrainCollision : SimplePhysicConstrain
{
    private int contactFrames = 0;
    private float[,] p = new float[3,12];
    private float[] b = new float[3];
    private int idA;
    private int idB;

    public override void Setup(Transform transform)
    {
        //throw new System.NotImplementedException();
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

    }


    public void AddConstrainCollision(ref int constrainCount, ref float[,] jacobi, ref float[] bias, int set)
    {
        //for (int set = 0; set < 3; ++set)
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
            constrainCount++;
        }
    }
}
