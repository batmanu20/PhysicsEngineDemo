using System.Collections;
using System.Collections.Generic;
using UnityEngine;

public class SimplePhysicConstrainConstant : SimplePhysicConstrain
{
    SimplePhysicConstrainConstant()
    {
        this.paramSetCount = 0;
    }

    public override void AddConstrain(ref int constrainCount, ref float[,] jacobi, ref float[] bias)
    {
    }
}