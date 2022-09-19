using System.Collections;
using System.Collections.Generic;
using UnityEngine;
using System;

public class SimplePhysicSolver : MonoBehaviour
{
    public static float _Beta = 0.5f;
    public static float _Cr = 1f;
    public static float _SlopP = 0.5f;
    public static float _SlopR = 50f;
    public static bool stable = true;
    public static float _Cf = 0.5f;
    public static float _CfPower = 0.5f;
    public static float _Epsilon = 1e-2f;
    public float gravity = -9.8f;
    public float beta = 0.5f;
    public float cr = 1f;
    public float cf = 0.5f;
    public float cfPower = 0.5f;
    public float slopP = 0.5f;
    public float slopR = 50f;
    public float epsilon = 1e-2f;
    public int maxSteps = 1;

    private List<SimplePhysicRigidBody> rigidBodies = new List<SimplePhysicRigidBody>();
    //public List<SimplePhysicConstrain> constrains = new List<SimplePhysicConstrain>();
    private int constrainCount = 0;
    private bool solveContactConstrain = false;
    // J * (-M) * (Jt) * lambda = -1 * J (Q1 / deltT + (_M)* (Fext)) = -1 * J(Vunsolved) / deltT ;
    // Then final deltV = (-M) * Jt * lambda;
    // where collision happens, right part = -1 * ( J(Vunsolved) / deltT  + bias)

    // Try right part = -1 * ( J(Vunsolved) + bias)
    // Try deltV = (-M) * Jt * lambda;
    private float[] lambda = new float[0];
    private float[] extendForceBuffer = new float[0];
    //private float[,] jacobi = new float[0, 0];
    // sumBuffer = res of J * (-M)
    private float[,] JinverMBuffer = new float[0, 0];
    // lambdaMtxBuffer = J * (-M) * (Jt)
    private float[,] lambdaMtxBuffer = new float[0, 0];
    // lambdaMtxBuffer = (J * (-M) * (Jt)).Inverse()
    private float[,] lambdaMtxInverseBuffer = new float[0, 0];
    // lambdaResBuffer = -1 * J (Q1 / deltTime + (_M)* (Fext))
    private float[] lambdaResBuffer = new float[0];
    // lambdaResHalfBuffer = Q1 / deltTime + (_M)* (Fext)
    // Try lambdaResHalfBuffer = Q1 + deltT * (_M)* (Fext)
    private float[] lambdaResHalfBuffer = new float[0];
    private float[] jtLambda = new float[0];
    private float lambdaSumN;
    private float[] lambdaSumT = new float[2];
    //private float lambdaSumB;
    private int contactCollisionCount = 0;
    // bias[i] != 0. collition happens.
    // bias = -1 * _Beta * max(distance - _Slop, 0) / deltT + Cr * max(Vc - _Slop, 0).
    // _Beta [0,1], _Cr[0, 1], _Slop: small limit of insection where rigidbody could be stable, 0.5mm should work;
    // C(collision) / dt = Vc >= 0  ----> Vc = (-Va - waXra + Vb + wbXrb) dot n
    // where ra is insection vector of a, and n is collision direction from a to b.
    // where jacob = [-n^T (-raXn)^T n^T (rbXn)^t]
    private float[] bias = new float[0];

    private float[,] jacobiCoefficients = new float[0, 0];
    private float[,] jacobipowers = new float[0, 0];
    private float[,] jacobibiases = new float[0, 0];

    private float[,] jacobi = new float[0, 0];

    private Vector3 vb1 = Vector3.one;
    private Vector3 vb2 = Vector3.one;
    private Vector3 vb3 = Vector3.one;
    private Vector4 v4b = Vector4.zero;

    public int RigistRigidbody(SimplePhysicRigidBody rigidbody)
    {
        this.rigidBodies.Add(rigidbody);
        
        return this.rigidBodies.Count - 1;
    }

    private void Initiate()
    {
        _Beta = beta;
        _Cr = cr;
        _SlopP = slopP;
        _SlopR = slopR;
        _Cf = cf;
        _CfPower = cfPower;
        _Epsilon = epsilon;
    }

    // Update is called once per frame
    void FixedUpdate()
    {
        if(Time.fixedDeltaTime > 0.033)
        {
            return;
        }
        Initiate();
        // Test force input
        foreach (var rigid in this.rigidBodies)
        {
            var force = rigid.testForce;
            rigid.AddForce();
        }
        this.ApplyForce();
        stable = false;
        int count = 0;
        while(!stable && count < maxSteps)
        {
            stable = true;
            this.DetectCollision();
            solveContactConstrain = true;
            this.ApplyContactNormalConstrain();
            this.SolveConstrain();
            this.ApplyContactTangentConstrain();
            this.SolveConstrain();
            solveContactConstrain = false;
            this.ApplyJointConstrain();
            this.SolveConstrain();
            count++;
        }
        this.UpdateRigidBody();
        if(count > 1)
        {
            Debug.Log($"Loop solving constrain count = {count}  {Time.frameCount}");
        }
    }

    public SimplePhysicRigidBody GetRigidbody(int index)
    {
        return this.rigidBodies[index];
    }

    private void ApplyForce()
    {
        this.InitiateArries();

        Vector3 force = Vector3.one;
        Vector3 torque = Vector3.one;

        for (int i = 0; i < rigidBodies.Count; ++i)
        {

            var rigid = rigidBodies[i];
            force = new Vector3(0, this.gravity * rigid.mass, 0);
            torque = Vector3.zero;
            for (int f = 0; f < rigid.posAndForces.Count; ++f)
            {
                force += rigid.posAndForces[f].force;
                //if(!rigid.posAndForces[f].localSpace)
                //{
                //    Matrix4x4 localTrans = Matrix4x4.TRS(rigid.transform.localPosition, rigid.transform.localRotation, rigid.transform.localScale);
                //    worldPos= localTrans.MultiplyPoint3x4(worldPos);
                //}
                torque += Vector3.Cross(rigid.posAndForces[f].position, rigid.posAndForces[f].force);
            }
            extendForceBuffer[6 * i + 0] = force.x;
            extendForceBuffer[6 * i + 1] = force.y;
            extendForceBuffer[6 * i + 2] = force.z;
            extendForceBuffer[6 * i + 3] = torque.x;
            extendForceBuffer[6 * i + 4] = torque.y;
            extendForceBuffer[6 * i + 5] = torque.z;
            //rigid.linearAccelaration = force / rigid.mass;
            //rigid.linearVelocity += rigid.linearAccelaration * Time.fixedDeltaTime;
            //rigid.angularAccelaration = Matrix4x4.Inverse(rigid.inertia).MultiplyPoint3x4(torque);
            //rigid.angularVelocity += rigid.angularAccelaration * Time.fixedDeltaTime;
        }

        for (int i = 0; i < this.rigidBodies.Count; ++i)
        {
            force.Set(extendForceBuffer[6 * i + 0], extendForceBuffer[6 * i + 1], extendForceBuffer[6 * i + 2]);
            var linearAcceleray = (force / this.rigidBodies[i].mass) * Time.fixedDeltaTime;
            //Debug.Log("Linear Acceleray = " + linearAcceleray * 100);
            this.rigidBodies[i].unsolvedLinearV = this.rigidBodies[i].ground ? Vector3.zero : 
                (this.rigidBodies[i].linearVelocity + (force / this.rigidBodies[i].mass) * Time.fixedDeltaTime);
            //Debug.Log("force" + solvedForce[0] + " " + solvedForce[1] + " " + solvedForce[2] + "  linear Velocity" + this.rigidBodies[i].linearVelocity);

            torque.Set(extendForceBuffer[6 * i + 3], extendForceBuffer[6 * i + 4], extendForceBuffer[6 * i + 5]);
            var deltAngularV = Matrix4x4.Inverse(this.rigidBodies[i].inertia).MultiplyPoint3x4(torque) * Time.fixedDeltaTime;
            this.rigidBodies[i].unsolvedAngularV =this.rigidBodies[i].ground ?
                Vector3.zero :
                (this.rigidBodies[i].angularVelocity + deltAngularV);

            this.rigidBodies[i].ApplyUnsolvedVelocity();
        }
    }

    private void DetectCollisonFar()
    {
        for (int i = 0; i < this.rigidBodies.Count; ++i)
        {
            this.rigidBodies[i].farCollisionTarget.Clear();
            for (int j = 0; j < this.rigidBodies.Count; ++j)
            {
                //if (this.rigidBodies[i].rigidIndex != this.rigidBodies[j].rigidIndex &&
                //    this.rigidBodies[i].ground == true)
                //    this.rigidBodies[i].farCollisionTarget.Add(this.rigidBodies[j].rigidIndex);
                if (this.rigidBodies[i].rigidIndex > this.rigidBodies[j].rigidIndex && !(this.rigidBodies[i].ground && this.rigidBodies[j].ground))
                    this.rigidBodies[i].farCollisionTarget.Add(this.rigidBodies[j].rigidIndex);

            }
        }
    }

    public virtual void DetectCollision()
    {
        DetectCollisonFar();
        for (int i = 0; i < this.rigidBodies.Count; ++i)
        {
            this.rigidBodies[i].DetectCollision();
        }
    }

    private void ApplyJointConstrain()
    {
        //this.constrains.Clear();
        this.constrainCount = 0;
        for (int i = 0; i < this.rigidBodies.Count; ++i)
        {
            var constrains = this.rigidBodies[i].staticConstrains;
            if (constrains != null && constrains.Length > 0)
            {
                for (int c = 0; c < constrains.Length; ++c)
                {
                    this.constrainCount += constrains[c].paramSetCount;
                }
            }
        }

        this.InitiateMtxBuffer();

        int constrainIndex = 0;
        for (int i = 0; i < this.rigidBodies.Count; ++i)
        {
            var constrains = this.rigidBodies[i].staticConstrains;
            if (constrains != null && constrains.Length > 0)
            {
                for (int c = 0; c < constrains.Length; ++c)
                {
                    constrains[c].AddConstrain(ref constrainIndex, ref this.jacobi, ref bias);
                }
            }
        }
    }

    private void ApplyContactNormalConstrain()
    {
        this.constrainCount = 0;
        int tempCount = 0;
        for (int i = 0; i < this.rigidBodies.Count; ++i)
        {
            var collisions = this.rigidBodies[i].collisionConstrains;
            if (collisions != null && collisions.Count > 0)
            {
                for (int c = 0; c < collisions.Count; ++c)
                {
                    tempCount++;
                    if (collisions[c].type == SimplePhysicConstrain.ConstrainType.ContactNormal)
                    {
                        this.constrainCount++;
                    }
                }
            }
        }

        contactCollisionCount = this.constrainCount;
        lambdaSumN = 0;
        this.InitiateMtxBuffer();

        int constrainIndex = 0;
        for (int i = 0; i < this.rigidBodies.Count; ++i)
        {
            var collisions = this.rigidBodies[i].collisionConstrains;
            if (collisions != null && collisions.Count > 0)
            {
                for (int c = 0; c < collisions.Count; ++c)
                {
                    if (collisions[c].type == SimplePhysicConstrain.ConstrainType.ContactNormal)
                    {
                        collisions[c].AddConstrain(ref constrainIndex, ref this.jacobi, ref bias);
                    }
                }
            }
        }
    }
    private void ApplyContactTangentConstrain()
    {
        this.constrainCount = 0;
        for (int i = 0; i < this.rigidBodies.Count; ++i)
        {
            var collisions = this.rigidBodies[i].collisionConstrains;
            if (collisions != null && collisions.Count > 0)
            {
                for (int c = 0; c < collisions.Count; ++c)
                {
                    if (collisions[c].type == SimplePhysicConstrain.ConstrainType.ContactTangent)
                    {
                        this.constrainCount++;
                    }
                }
            }
        }
        this.lambdaSumT[0] = 0;
        this.lambdaSumT[1] = 0;
        this.InitiateMtxBuffer();

        int constrainIndex = 0;
        for (int i = 0; i < this.rigidBodies.Count; ++i)
        {
            var collisions = this.rigidBodies[i].collisionConstrains;
            if (collisions != null && collisions.Count > 0)
            {
                for (int c = 0; c < collisions.Count; ++c)
                {
                    if (collisions[c].type == SimplePhysicConstrain.ConstrainType.ContactTangent)
                    {
                        collisions[c].AddConstrain(ref constrainIndex, ref this.jacobi, ref bias);
                    }
                }
            }
        }
    }
    private void ApplyConstrain()
    {
        //this.constrains.Clear();
        this.constrainCount = 0;
        for (int i = 0; i < this.rigidBodies.Count; ++i)
        {
            var constrains = this.rigidBodies[i].staticConstrains;
            if(constrains != null && constrains.Length > 0)
            {
                for (int c = 0; c < constrains.Length; ++c)
                {
                    this.constrainCount += constrains[c].paramSetCount;
                }
            }
        }

        for (int i = 0; i < this.rigidBodies.Count; ++i)
        {
            var constrains = this.rigidBodies[i].collisionConstrains;
            if (constrains != null && constrains.Count > 0)
            {
                this.constrainCount += this.rigidBodies[i].collisionConstrains.Count;
            }
        }

        this.InitiateMtxBuffer();

        int constrainIndex = 0;
        for (int i = 0; i < this.rigidBodies.Count; ++i)
        {
            var constrains = this.rigidBodies[i].staticConstrains;
            //int rigidBodyIndex = this.rigidBodies[i].dataIndex;
            if (constrains != null && constrains.Length > 0)
            {
                for (int c = 0; c < constrains.Length; ++c)
                {
                    constrains[c].AddConstrain(ref constrainIndex, ref this.jacobi, ref bias);
                }
            }
            var collisions = this.rigidBodies[i].collisionConstrains;
            if (collisions != null && collisions.Count > 0)
            {
                for (int c = 0; c < collisions.Count; ++c)
                {
                    collisions[c].AddConstrain(ref constrainIndex, ref this.jacobi, ref bias);
                }
            }
        }
    }

    private void SolveConstrain()
    {
        // velocity2 = velocity1 + deltT * (-M)(Jt * lambda + extendForce)
        int constrainCount = this.constrainCount;
        if (constrainCount != 0)
        {
            this.CalculateLambda();
            this.CalculateSolvedForce();

            for (int i = 0; i < this.rigidBodies.Count; ++i)
            {
                var oriL = this.rigidBodies[i].unsolvedLinearV;
                var oriA = this.rigidBodies[i].unsolvedAngularV;
                vb1.Set(jtLambda[6 * i + 0], jtLambda[6 * i + 1], jtLambda[6 * i + 2]);
                this.rigidBodies[i].unsolvedLinearV = (this.rigidBodies[i].ground ? Vector3.zero :
                    this.rigidBodies[i].unsolvedLinearV + (vb1 / this.rigidBodies[i].mass));
                vb1.Set(jtLambda[6 * i + 3], jtLambda[6 * i + 4], jtLambda[6 * i + 5]);
                this.rigidBodies[i].unsolvedAngularV = this.rigidBodies[i].ground ? Vector3.zero :
                    this.rigidBodies[i].unsolvedAngularV + Matrix4x4.Inverse(this.rigidBodies[i].inertia).MultiplyPoint3x4(vb1);

                float solvedJV = 0;
                vb1.Set(jacobi[0, 6 * i + 0], jacobi[0, 6 * i + 1], jacobi[0, 6 * i + 2]);
                solvedJV += Vector3.Dot(vb1, this.rigidBodies[i].unsolvedLinearV);
                vb2.Set(jacobi[0, 6 * i + 3], jacobi[0, 6 * i + 4], jacobi[0, 6 * i + 5]);
                solvedJV += Vector3.Dot(vb2, this.rigidBodies[i].unsolvedAngularV);

                if (!this.rigidBodies[i].ground)
                {
                    //Debug.Log($"unsolvedV {oriL} {this.rigidBodies[i].unsolvedLinearV}  angulr {oriA} {this.rigidBodies[i].unsolvedAngularV} solvedJV {solvedJV}");
                    //this.rigidBodies[i].DumpSolvedBias();
                }

                this.rigidBodies[i].ApplyUnsolvedVelocity();
            }
        }
        //else
        //{
        //    for (int i = 0; i < this.rigidBodies.Count; ++i)
        //    {
        //        this.rigidBodies[i].unsolvedLinearV = this.rigidBodies[i].ground ? Vector3.zero :
        //            this.rigidBodies[i].unsolvedLinearV;
        //        this.rigidBodies[i].unsolvedAngularV = this.rigidBodies[i].ground ? Vector3.zero :
        //            this.rigidBodies[i].unsolvedAngularV;
        //    }
        //    solvedForce = extendForceBuffer;
        //}

        // calculate deltV of jv+d;
        //for (int i = 0; i < this.rigidBodies.Count; ++i)
        //{
        //    for (int j = 0; j < 3; ++j)
        //    {
        //        vb1[j] = 0;
        //        for (int c = 0; c < this.constrainCount; ++c)
        //        {
        //            vb1[j] += jacobi[c, 6 * i] * lambda[c];
        //        }
        //    }

        //    Vector3 deltV = vb1 / rigidBodies[i].mass;
        //    rigidBodies[i].linearVelocity += this.rigidBodies[i].ground ? Vector3.zero : deltV;
        //    // calculate deltV;
        //    for (int j = 0; j < 3; ++j)
        //    {
        //        vb1[j] = 0;
        //        for (int c = 0; c < this.constrainCount; ++c)
        //        {
        //            vb2.Set(jacobi[c, 6 * i + 3], jacobi[c, 6 * i + 4], jacobi[c, 6 * i + 5]);
        //            vb1[j] += Vector3.Dot(this.rigidBodies[i].inertia.inverse.GetRow(j), vb2) * lambda[c];
        //        }
        //    }
        //    rigidBodies[i].angularVelocity += this.rigidBodies[i].ground ? Vector3.zero : vb1;
        //}
        //return; 

        Vector3 force = Vector3.one;
        Vector3 torque = Vector3.one;
        // Calculate new Speed
        //for (int i = 0; i < this.rigidBodies.Count; ++i)
        //{
        //    force.Set(solvedForce[6 * i + 0], solvedForce[6 * i + 1], solvedForce[6 * i + 2]);
        //    var linearAcceleray = (force / this.rigidBodies[i].mass) * Time.fixedDeltaTime;
        //    //Debug.Log("Linear Acceleray = " + linearAcceleray * 100);
        //    this.rigidBodies[i].linearVelocity += (this.rigidBodies[i].ground ? Vector3.zero : (force / this.rigidBodies[i].mass)) * Time.fixedDeltaTime;
        //    //Debug.Log("force" + solvedForce[0] + " " + solvedForce[1] + " " + solvedForce[2] + "  linear Velocity" + this.rigidBodies[i].linearVelocity);

        //    torque.Set(solvedForce[6 * i + 3], solvedForce[6 * i + 4], solvedForce[6 * i + 5]);
        //    this.rigidBodies[i].angularVelocity += this.rigidBodies[i].ground ? Vector3.zero : Matrix4x4.Inverse(this.rigidBodies[i].inertia).MultiplyPoint3x4(torque) * Time.fixedDeltaTime;
        //}
    }

    private void InitiateArries()
    {
        var length = this.rigidBodies.Count * 6;
        if (extendForceBuffer.Length != length)
        {
            extendForceBuffer = new float[length];
        }
        if (lambdaResHalfBuffer.Length != length)
        {
            lambdaResHalfBuffer = new float[length];
        }
        if (jtLambda.Length != length)
        {
            jtLambda = new float[length];
        }
    }

    private void InitiateMtxBuffer()
    {
        var length = this.constrainCount;
        if (lambda.Length != length)
        {
            lambda = new float[length];
        }
        if (lambdaResBuffer.Length != length)
        {
            lambdaResBuffer = new float[length];
        }
        if (lambdaMtxBuffer.Length != length * length)
        {
            lambdaMtxBuffer = new float[length, length];
        }
        if (lambdaMtxInverseBuffer.Length != length * length)
        {
            lambdaMtxInverseBuffer = new float[length, length];
        }
        if (bias.Length != this.constrainCount)
        {
            bias = new float[this.constrainCount];
        }
        if (JinverMBuffer.Length != this.constrainCount)
        {
            JinverMBuffer = new float[this.constrainCount, this.rigidBodies.Count * 6];
        }
        if (jacobiCoefficients.Length != this.constrainCount * this.rigidBodies.Count * 6)
        {
            jacobiCoefficients = new float[this.constrainCount, this.rigidBodies.Count * 6];
        }
        if (jacobibiases.Length != this.constrainCount * this.rigidBodies.Count * 6)
        {
            jacobibiases = new float[this.constrainCount, this.rigidBodies.Count * 6];
        }
        if (jacobipowers.Length != this.constrainCount * this.rigidBodies.Count * 6)
        {
            jacobipowers = new float[this.constrainCount, this.rigidBodies.Count * 6];
        }
        if (jacobi.Length != this.constrainCount * this.rigidBodies.Count * 6)
        {
            jacobi = new float[this.constrainCount, this.rigidBodies.Count * 6];
        }
    }

    private void UpdateRigidBody()
    {
        for(int i = 0; i< rigidBodies.Count; ++i)
        {
            rigidBodies[i].ApplySimulation();
        }
    }

    private void CalculateLambda()
    {
        int constrainCount = this.constrainCount;
        int itemlength = this.rigidBodies.Count * 6;

        // Calculate two buffers
        // Calculate J * (-M) * (Jt)
        // For J * (-M)
        for (int i = 0; i < constrainCount; ++i)
        {
            for (int j = 0; j < this.rigidBodies.Count; ++j)
            {
                for (int k = 0; k < 3; ++k)
                {
                    JinverMBuffer[i, 6 * j + k] = jacobi[i, 6 * j + k] / this.rigidBodies[j].mass;
                }
                vb1.Set(jacobi[i, 6 * j + 3], jacobi[i, 6 * j + 4], jacobi[i, 6 * j + 5]);
                for (int k = 0; k < 3; ++k)
                {
                    v4b = this.rigidBodies[j].inertia.inverse.GetColumn(k);
                    float res = Vector3.Dot(vb1, v4b);
                    JinverMBuffer[i, 6 * j + 3 + k] = res;
                }
            }
        }
        // For {J * (_M)} *{Jt}
        for (int i = 0; i < constrainCount; ++i)
        {
            for (int j = 0; j < constrainCount; ++j)
            {
                lambdaMtxBuffer[i, j] = 0;
                for (int k = 0; k < itemlength; ++k)
                {
                    lambdaMtxBuffer[i, j] += JinverMBuffer[i, k] * jacobi[j, k];
                }
            }
        }

        if(constrainCount > 0 )
        {
            //Debug.Log($"lambdaMtxBuffer {lambdaMtxBuffer[0, 0]}");
        }
        // Calculate lambdaResBuffer = -1 * J (Q1 / deltTime + (_M)* (Fext))
        // lambdaResHalfBuffer = Q1 / deltTime + (_M)* (Fext)
        // mXn Matrix: m = constrainCount; n = 6 * rigidCount

        // Try Calculate lambdaResBuffer = -1 * J (Q1 + deltTime * (_M)* (Fext))
        // Try lambdaResHalfBuffer = Q1 + deltTime * (_M)* (Fext)
        for (int i = 0; i < this.rigidBodies.Count; ++i)
        {
            for (int k = 0; k < 3; ++k)
            {
                //lambdaResHalfBuffer[6 * i + k] = this.rigidBodies[i].linearVelocity[k] / Time.fixedDeltaTime;
                //lambdaResHalfBuffer[6 * i + k] += extendForceBuffer[6 * i + k] / this.rigidBodies[i].mass;
                lambdaResHalfBuffer[6 * i + k] = this.rigidBodies[i].unsolvedLinearV[k];
            }
            vb1.Set(extendForceBuffer[6 * i + 3], extendForceBuffer[6 * i + 4], extendForceBuffer[6 * i + 5]);
            vb3 = this.rigidBodies[i].inertia.inverse.MultiplyPoint3x4(vb1);
            for (int k = 0; k < 3; ++k)
            {
                //lambdaResHalfBuffer[6 * i + 3 + k] = this.rigidBodies[i].angularVelocity[k] / Time.fixedDeltaTime;
                //lambdaResHalfBuffer[6 * i + 3 + k] += vb3[k];
                lambdaResHalfBuffer[6 * i + 3 + k] = this.rigidBodies[i].unsolvedAngularV[k];
            }
        }

        for (int i = 0; i < this.constrainCount; ++i)
        {
            lambdaResBuffer[i] = 0;
            if (bias[i] != 0)
            {
                //Debug.Log("JV = " + lambdaResBuffer[i] + bias[i]);
                int a = 0;
            }
            for (int j = 0; j < this.rigidBodies.Count * 6; ++j)
            {
                lambdaResBuffer[i] += jacobi[i, j] * lambdaResHalfBuffer[j] ;
            }
            if(bias[i] != 0)
            {
                //Debug.Log("JV = " + lambdaResBuffer[i] + " bias= " + bias[i]);
            }
            //lambdaResBuffer[i] *= _Beta;
            lambdaResBuffer[i] += bias[i];
            //lambdaResBuffer[i] = Math.Min(0, lambdaResBuffer[i]);
            lambdaResBuffer[i] *= -1;
        }

        CalculateInverseJMJ(ref this.lambdaMtxBuffer, ref this.lambdaMtxInverseBuffer);

        for (int i = 0; i < constrainCount; ++i)
        {
            float tempLambda = 0;
            for (int j = 0; j < constrainCount; ++j)
            {
                tempLambda += lambdaMtxInverseBuffer[i, j] * (lambdaResBuffer[j]);
            }
            if(!solveContactConstrain)
            {
                lambda[i] = tempLambda;
            }
            else
            {
                if (bias[i] != 0)
                {
                    float oriLamda = lambdaSumN;
                    lambdaSumN += tempLambda;
                    lambdaSumN = Math.Max(0, lambdaSumN);
                    lambda[i] = lambdaSumN - oriLamda;
                }
                else
                {
                    var id = i % 2;
                    float oriLamda = lambdaSumT[id];
                    lambdaSumT[id] += tempLambda;
                    lambdaSumT[id] = Math.Clamp(lambdaSumT[id], -lambdaSumN * 1.414f * cf, lambdaSumN * 1.414f * cf);
                    lambda[i] = lambdaSumT[id] - oriLamda;
                    //lambda[i] = tempLambda;
                    //lambda[i] = Math.Min(tempLambda, lamdaNSum * 0.5f);
                }
            }

            //Debug.Log($"lambda{i}={lambda[i]} temp{tempLambda} {Time.frameCount}");
        }
        if (constrainCount > 0)
        {
        }
    }

    private void CalculateSolvedForce()
    {
        for(int i = 0; i < this.rigidBodies.Count * 6; ++i)
        {
            jtLambda[i] = 0;
            for(int j = 0; j < this.constrainCount; ++j)
            {
                jtLambda[i] += jacobi[j, i] * lambda[j];
            }
        }

    }

    private float GetJacobi(int i, int j)
    {
        return jacobiCoefficients[i, j] * Mathf.Pow(this.rigidBodies[i / 6].GetP(i), jacobipowers[i, j]) + jacobibiases[i, j];
    }

    private static void CalculateInverseJMJ(ref float[,] input, ref float[,] res)
    {
        int n;
        n = input.GetLength(0);
        float[,] C = new float[n, 2 * n];
        for (int i = 0; i < n; i++)
            for (int j = 0; j < n; j++)
                C[i, j] = input[i, j];
        for (int i = 0; i < n; i++)
            C[i, i + n] = 1;
        for (int k = 0; k < n; k++)
        {
            float max = Math.Abs(C[k, k]);
            int ii = k;
            for (int m = k + 1; m < n; m++)
                if (max < Math.Abs(C[m, k]))
                {
                    max = Math.Abs(C[m, k]);
                    ii = m;
                }
            for (int m = k; m < 2 * n; m++)
            {
                if (ii == k) break;
                float c;
                c = C[k, m];
                C[k, m] = C[ii, m];
                C[ii, m] = c;
            }
            if (C[k, k] != 1)
            {
                float bs = C[k, k];
                if (bs == 0)
                {
                    Console.WriteLine("求逆错误！结果可能不正确！");
                    break;
                    //return null;
                }
                C[k, k] = 1;
                for (int p = k + 1; p < n * 2; p++)
                {
                    C[k, p] /= bs;
                }
            }

            for (int q = k + 1; q < n; q++)
            {
                float bs = C[q, k];
                for (int p = k; p < n * 2; p++)
                {
                    C[q, p] -= bs * C[k, p];
                }
            }
        }
        for (int q = n - 1; q > 0; q--)
        {
            for (int k = q - 1; k > -1; k--)
            {
                float bs = C[k, q];
                for (int m = k + 1; m < 2 * n; m++)
                {
                    C[k, m] -= bs * C[q, m];
                }
            }
        }

        for (int i = 0; i < n; i++)
            for (int j = 0; j < n; j++)
                res[i, j] = C[i, j + n];
    }
}
