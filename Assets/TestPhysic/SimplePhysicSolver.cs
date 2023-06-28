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
    public static float _Epsilon = 1e-2f;
    public float gravity = -9.8f;
    public float beta = 0.5f;
    public float cr = 1f;
    public float cf = 0.5f;
    public float slopP = 0.5f;
    public float slopR = 50f;
    public float epsilon = 1e-2f;
    public int maxSteps = 1;

    private List<SimplePhysicRigidBody> rigidBodies = new List<SimplePhysicRigidBody>();
    private SimplePhysicConstrainCollision collision;
    private SimplePhysicConstrain constrain;
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
    // for J * (-M) * (Jt) different constrain will not influence each other.
    // so for lambda[i]  J * (-M) * (Jt)  = J[i] * (-M) * (Jt[i]) 
    // sumBuffer = res of J * (-M)
    private float[,] JinverMBuffer = new float[0, 0];
    // lambdaMtxBuffer = J * (-M) * (Jt)
    private float[] lambdaMtxBuffer = new float[0];
    // lambdaMtxBuffer = (J * (-M) * (Jt)).Inverse()
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
    
    private bool contactNormal = false;
    private bool solved = false;

    private const int ConstrainBufferSize = 2;
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
        lambdaSumT = new float[2];
        int count = 0;
        solved = false;
        this.DetectCollision();
        this.ApplyConstrainWarmups();
        while (count < maxSteps && !solved)
        {
            this.SolveCollisionConstrain();
            this.SolveJointConstrain();
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
                torque += Vector3.Cross(rigid.posAndForces[f].position, rigid.posAndForces[f].force);
            }
            extendForceBuffer[6 * i + 0] = force.x;
            extendForceBuffer[6 * i + 1] = force.y;
            extendForceBuffer[6 * i + 2] = force.z;
            extendForceBuffer[6 * i + 3] = torque.x;
            extendForceBuffer[6 * i + 4] = torque.y;
            extendForceBuffer[6 * i + 5] = torque.z;
        }

        for (int i = 0; i < this.rigidBodies.Count; ++i)
        {
            force.Set(extendForceBuffer[6 * i + 0], extendForceBuffer[6 * i + 1], extendForceBuffer[6 * i + 2]);
            var linearAcceleray = (force / this.rigidBodies[i].mass) * Time.fixedDeltaTime;
            torque.Set(extendForceBuffer[6 * i + 3], extendForceBuffer[6 * i + 4], extendForceBuffer[6 * i + 5]);
            this.rigidBodies[i].ApplyUnsolvedForce(force, torque);
        }
    }

    private void DetectCollisonFar()
    {
        for (int i = 0; i < this.rigidBodies.Count; ++i)
        {
            this.rigidBodies[i].farCollisionTarget.Clear();
            for (int j = 0; j < this.rigidBodies.Count; ++j)
            {
                if (this.rigidBodies[i].rigidIndex > this.rigidBodies[j].rigidIndex 
                    && !(this.rigidBodies[i].ground && this.rigidBodies[j].ground))
                {
                    this.rigidBodies[i].farCollisionTarget.Add(this.rigidBodies[j].rigidIndex);
                }
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

    private void SolveCollisionConstrain()
    {
        solveContactConstrain = true;
        for (int i = 0; i < this.rigidBodies.Count; ++i)
        {
            var collisions = this.rigidBodies[i].collisionConstrains;
            if (collisions != null && collisions.Count > 0)
            {
                foreach (var c in collisions)
                {
                    this.collision = c.Value;
                    this.constrainCount = 3;
                    int constrainIndex = 0;
                    this.InitiateMtxBuffer();
                    c.Value.AddConstrain(ref constrainIndex, ref this.jacobi, ref bias);
                    this.SolveConstrain();
                }
            }
        }
    }

    private void SolveJointConstrain()
    {
        solveContactConstrain = false;
        for (int i = 0; i < this.rigidBodies.Count; ++i)
        {
            var constrains = this.rigidBodies[i].staticConstrains;
            if (constrains != null && constrains.Count > 0)
            {
                for (int c = 0; c < constrains.Count; ++c)
                {
                    int constrainIndex = 0;
                    this.constrain = constrains[c];
                    this.constrainCount = this.constrain.paramSetCount;
                    this.InitiateMtxBuffer();
                    constrains[c].AddConstrain(ref constrainIndex, ref this.jacobi, ref bias);
                    this.SolveConstrain();
                }
            }
        }

        this.SolveConstrain();
    }

    private void ApplyConstrainWarmups()
    {
        // Collect all constrains first.
        this.constrainCount = 0;
        List<SimplePhysicConstrainCollision> warmupCollisions = new List<SimplePhysicConstrainCollision>();
        for (int i = 0; i < this.rigidBodies.Count; ++i)
        {
            var collisions = this.rigidBodies[i].collisionConstrains;
            if (collisions != null && collisions.Count > 0)
            {
                foreach (var c in collisions)
                {
                    warmupCollisions.Add(c.Value);
                }
            }
        }

        foreach(var collision in warmupCollisions)
        {
            collision.Warmup(this);
        }
    }

    private void SolveConstrain()
    {
        // velocity2 = velocity1 + deltT * (-M)(Jt * lambda + extendForce)
        if (this.constrainCount != 0)
        {
            this.CalculateLambda();
            this.CalculateSolvedForce();

            for (int i = 0; i < this.rigidBodies.Count; ++i)
            {
                vb1.Set(jtLambda[6 * i + 0], jtLambda[6 * i + 1], jtLambda[6 * i + 2]);
                vb2.Set(jtLambda[6 * i + 3], jtLambda[6 * i + 4], jtLambda[6 * i + 5]);

                this.rigidBodies[i].ApplyUnsolvedVelocity(vb1, vb2);
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

        float lambdaSum = 0;
        for (int i = 0; i < constrainCount; ++i)
        {
            var reverse = (lambdaMtxBuffer[i] > 1e-5) ? 1.0f / lambdaMtxBuffer[i] : 0;
            var tempLambda = reverse * lambdaResBuffer[i];
            if(!solveContactConstrain)
            {
                lambda[i] = tempLambda;
            }
            else
            {
                var contactNormal = i == 0;
                if (contactNormal)
                {
                    float oriLamda = lambdaSumN;
                    lambdaSumN += tempLambda;
                    lambdaSumN = Math.Max(0, lambdaSumN);
                    lambda[i] = lambdaSumN - oriLamda;
                    this.collision.lambdaN = lambda[i];
                }
                else
                {
                    // Clamp tangent lambda with lambdaN of collision.
                    var lambdaN = Math.Abs(this.collision.lambdaN);
                    lambda[i] = Math.Clamp(tempLambda, -lambdaN * 1.414f * cf, lambdaN * 1.414f * cf);
                    //lambda[i] = tempLambda;
                    //lambda[i] = Math.Min(tempLambda, lamdaNSum * 0.5f);
                }
            }

            lambdaSum += lambda[i];
        }
        if(lambdaSum == 0)
        {
            solved = true;
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
