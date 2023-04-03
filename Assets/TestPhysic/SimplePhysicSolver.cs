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
    private float[] lambdaMtxBuffer = new float[0];
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
    private Dictionary<Vector2Int, float> warmStartingLambda = new Dictionary<Vector2Int, float>();

    private Vector3 vb1 = Vector3.one;
    private Vector3 vb2 = Vector3.one;
    private Vector3 vb3 = Vector3.one;
    private Vector4 v4b = Vector4.zero;
    
    private bool contactNormal = false;

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

        // Some global state
        lambdaSumN = 0;

        int count = 0;
        while (count < maxSteps)
        {
            this.DetectCollision();
            solveContactConstrain = true;
            contactNormal = true;
            this.ApplyContactNormalConstrain();
            this.SolveConstrain();
            contactNormal = false;
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
            if (constrains != null && constrains.Count > 0)
            {
                for (int c = 0; c < constrains.Count; ++c)
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
            if (constrains != null && constrains.Count > 0)
            {
                for (int c = 0; c < constrains.Count; ++c)
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
        //if (this.constrainCount > 0)
        //    Debug.Log($"normal collision count {this.constrainCount}");

        contactCollisionCount = this.constrainCount;
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
        //if(this.constrainCount > 0)
        //    Debug.Log($"tangent collision count {this.constrainCount}");
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
                    //Debug.Log($"name{this.rigidBodies[i].name} unsolvedV {oriL} {this.rigidBodies[i].unsolvedLinearV}  angulr {oriA} {this.rigidBodies[i].unsolvedAngularV} solvedJV {solvedJV}");
                }

                this.rigidBodies[i].ApplyUnsolvedVelocity();
            }
        }
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
        if (lambdaMtxBuffer.Length != length)
        {
            lambdaMtxBuffer = new float[length];
        }
        if (lambdaMtxInverseBuffer.Length != length * length)
        {
            lambdaMtxInverseBuffer = new float[length, length];
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

        // clear every step
        bias = new float[this.constrainCount];
        jacobi = new float[this.constrainCount, this.rigidBodies.Count * 6];
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

        if (constrainCount > 3)
        {
            //Debug.Log("multi collision");
            int ddsd = 0;
        }

        // For {J * (_M)} *{Jt}
        for (int i = 0; i < constrainCount; ++i)
        {
            lambdaMtxBuffer[i] = 0;
            for (int k = 0; k < itemlength; ++k)
            {
                lambdaMtxBuffer[i] += JinverMBuffer[i, k] * jacobi[i, k];
            }
        }

        if(constrainCount > 0 )
        {
            //Debug.Log($"lambdaMtxBuffer {lambdaMtxBuffer[0, 0]}");
        }

        // Calculate lambdaResBuffer = -1 * J (Q1 + deltTime * (_M)* (Fext))
        // lambdaResHalfBuffer = Q1 + deltTime * (_M)* (Fext)
        for (int i = 0; i < this.rigidBodies.Count; ++i)
        {
            for (int k = 0; k < 3; ++k)
            {
                lambdaResHalfBuffer[6 * i + k] = this.rigidBodies[i].unsolvedLinearV[k];
            }
            vb1.Set(extendForceBuffer[6 * i + 3], extendForceBuffer[6 * i + 4], extendForceBuffer[6 * i + 5]);
            vb3 = this.rigidBodies[i].inertia.inverse.MultiplyPoint3x4(vb1);
            for (int k = 0; k < 3; ++k)
            {
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

        //CalculateInverseJMJ(ref this.lambdaMtxBuffer, ref this.lambdaMtxInverseBuffer);

        for (int i = 0; i < constrainCount; ++i)
        {
            float tempLambda = 0;
            for (int j = 0; j < constrainCount; ++j)
            {
                var reverse = (i == j && lambdaMtxBuffer[i] > 1e-5) ? 1.0f / lambdaMtxBuffer[i] : 0;
                tempLambda += reverse * (lambdaResBuffer[j]);
            }
            if(!solveContactConstrain)
            {
                lambda[i] = tempLambda;
            }
            else
            {
                if (contactNormal)
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
}
