using System;
using System.Collections;
using System.Collections.Generic;
using Unity.VisualScripting;
using UnityEngine;
using Mathf = UnityEngine.Mathf;

public class SimplePhysicRigidBody : MonoBehaviour
{
    public List<TestForceInput> testForce;
    private List<Vector3> collisionGizmosSimplex = new List<Vector3>();
    private List<Vector3> collisionGizmosContact = new List<Vector3>();
    private Vector3 collisionGizmosNearest;
    private Vector3 debugAngular = new Vector3();
    private Vector3 historyPos;
    private float historyRot;
    private int historyCount = 0;

    [Serializable]
    public struct TestForceInput
    {
        [SerializeField]
        public Vector3 localPoint;
        [SerializeField]
        public Vector3 worldForce;
    }

    public enum RigidbodyType 
    {
        box,
        upFace,
        sphere,
    }

    public struct CollisionResult
    {
        public bool collision;
        public Vector3 contactDirA;
        public Vector3 contactDirB;
        public Vector3 normalDirection;
        public Vector3 tangentDirection;
        public float insertion;
    }

    private SimplePhysicSolver solver;
    public int rigidIndex = -1;
    public RigidbodyType type;
    public bool ground = false;
    public bool collider = true;
    public bool applyCollision = true;

    public bool[] freeFlag = new bool[6] { false, false, false, false, false, false };
    public Vector3 linearVelocity;
    public Vector3 angularVelocity;
    public float mass;
    public Matrix4x4 inertia;

    public Vector3 cacheDeltRotation;
    public Vector3 unsolvedLinearV;
    public Vector3 unsolvedAngularV;
    public Vector3 unsolvedPosition;
    public Matrix4x4 unsolvedRotation;
    public List<int> farCollisionTarget = new List<int>();

    public List<SimplePhysicConstrain> staticConstrains = new List<SimplePhysicConstrain>();
    //public List<SimplePhysicConstrainCollision> collisionConstrains = new List<SimplePhysicConstrainCollision>();
    public Dictionary<int, SimplePhysicConstrainCollision> collisionConstrains = new Dictionary<int, SimplePhysicConstrainCollision>();
    private CollisionResult resultCache;
    private Simplex debugSimplex;
    [HideInInspector]
    public bool debugImpulse = false;
    public bool DebugSimplexEnabled;
    public bool debugSupport = false;
    public SimplePhysicRigidBody supportTarget;
    public int debugSupportCount = 20;

    public struct Force
    {
        public Vector3 position;
        public Vector3 force;
        public Force(Vector3 p, Vector3 f)
        {
            position = p;
            force = f;
        }
    }

    public List<Force> posAndForces = new List<Force>(); 

    private void OnEnable()
    {
        var parent = transform.parent;
        while (parent != null)
        {
            solver = parent.GetComponent<SimplePhysicSolver>();
            if (solver != null) break;
            parent = parent.parent;
        }

        if(solver!= null)
        {
            rigidIndex = solver.RigistRigidbody(this);
        }

        this.CalculateParams();
        if(this.ground)
        {
            this.mass = int.MaxValue;
            this.inertia = Matrix4x4.TRS(Vector3.zero, Quaternion.identity, Vector3.one * int.MaxValue);
        }
        this.InitiateType();
    }

    public virtual void InitiateType()
    {
        this.type = RigidbodyType.box;
    }

    public void AddForce(bool localPoint, Vector3 point, Vector3 force)
    {
        this.posAndForces.Clear();
        Matrix4x4 localMatrix = Matrix4x4.TRS(Vector3.zero, this.transform.localRotation, this.transform.localScale);
        if (localPoint)
        {
            point = localMatrix.MultiplyPoint3x4(point);
        }

        posAndForces.Add(new Force(point, force));
    }

    public void AddForce()
    {
        this.posAndForces.Clear();
        Matrix4x4 localMatrix = Matrix4x4.TRS(Vector3.zero, this.transform.localRotation, this.transform.localScale);
        if (testForce != null)
        {
            for(int i = 0; i< this.testForce.Count; ++i)
            {
                var point = localMatrix.MultiplyPoint3x4(testForce[i].localPoint);
                posAndForces.Add(new Force(point, testForce[i].worldForce));

            }
        }   
    }

    virtual public void CalculateParams()
    {
        this.mass = 1;
        this.inertia = Matrix4x4.identity;
    }

    virtual public Vector3 SupportFunction(ref Vector3 dir)
    {
        throw new Exception($"can not find override method of support function with this rigid body: {this.name}");
    }

    protected Vector3 WorldTDiroLocal(Vector3 dir)
    {
        return this.unsolvedRotation.inverse.MultiplyPoint3x4(dir);
    }

    protected Vector3 LocalDirToWorld(Vector3 dir)
    {
        return this.unsolvedRotation.MultiplyPoint3x4(dir) + this.unsolvedPosition;
    }

    public void DetectCollision()
    {
        for (int i = 0; i < farCollisionTarget.Count; ++i)
        {
            if(this.collider && this.solver.GetRigidbody(farCollisionTarget[i]).collider)
            {
                this.DetectCollision(this.solver.GetRigidbody(farCollisionTarget[i]));
            }
        }
    }

    public virtual void AppyCollision(CollisionResult result, SimplePhysicRigidBody target, SimplePhysicConstrainCollision constrain)
    {
        float vc = Vector3.Dot(
                - this.unsolvedLinearV
                - Vector3.Cross(this.unsolvedAngularV, result.contactDirA)
                + target.unsolvedLinearV
                + Vector3.Cross(target.unsolvedAngularV, result.contactDirB)
            , result.normalDirection);

        float b1 = -1 * SimplePhysicSolver._Beta
            * Mathf.Max(result.insertion - SimplePhysicSolver._SlopP, 0) / Time.fixedDeltaTime;
        float b2 = SimplePhysicSolver._Cr * Math.Sign(vc) * Mathf.Max(Math.Abs(vc) - SimplePhysicSolver._SlopR, 0);
        float b = b1 + b2;
        Vector3 raXn = Vector3.Cross(result.contactDirA, result.normalDirection);
        Vector3 rbXn = Vector3.Cross(result.contactDirB, result.normalDirection);
        //Debug.Log("b1 = " + b1 + " b2 = " + b2 + " vc =" + vc + " -va = " + (-this.linearVelocity) + " dot " + Vector3.Dot(-this.linearVelocity, result.normalDirection)
        //     + "raXN = " + raXn + "rbxn" + rbXn);
        if (vc > 0)
        {
            //return;
            var v1 = -Vector3.Cross(this.unsolvedAngularV, result.contactDirA);
            int a = 0;
        }

        float[] p = new float[12]
        {
            -result.normalDirection.x,
            -result.normalDirection.y,
            -result.normalDirection.z,
            -raXn.x,
            -raXn.y,
            -raXn.z,
            result.normalDirection.x,
            result.normalDirection.y,
            result.normalDirection.z,
            rbXn.x,
            rbXn.y,
            rbXn.z,
        };


        constrain.SetParameter(p, b, this.rigidIndex, target.rigidIndex, 0);

        var vecT = Vector3.Normalize(this.unsolvedLinearV - target.unsolvedLinearV);
        if (Vector3.Dot(vecT, Vector3.forward) == 1)
        {
            vecT = Vector3.right;
        }
        var vecB = Vector3.Normalize(Vector3.Cross(result.normalDirection, vecT));
        vecT = Vector3.Normalize(Vector3.Cross(vecB, result.normalDirection));
        var raXt = Vector3.Cross(result.contactDirA, vecT);
        var rbXt = Vector3.Cross(result.contactDirB, vecT);
        p = new float[12]
        {
            -vecT.x,
            -vecT.y,
            -vecT.z,
            -raXt.x,
            -raXt.y,
            -raXt.z,
            vecT.x,
            vecT.y,
            vecT.z,
            rbXt.x,
            rbXt.y,
            rbXt.z,
        };
        vc = Vector3.Dot(
            -this.unsolvedLinearV
                - Vector3.Cross(this.unsolvedAngularV, vecT)
                + target.unsolvedLinearV
                + Vector3.Cross(target.unsolvedAngularV, vecT)
            , vecT);
        b2 = -(1 - SimplePhysicSolver._Cf) * Math.Sign(vc) * Mathf.Max(Math.Abs(vc) - SimplePhysicSolver._SlopR, 0);

        constrain.SetParameter(p, b2, this.rigidIndex, target.rigidIndex, 1);

        raXt = Vector3.Cross(result.contactDirA, vecB);
        rbXt = Vector3.Cross(result.contactDirB, vecB);
        p = new float[12]
        {
            -vecB.x,
            -vecB.y,
            -vecB.z,
            -raXt.x,
            -raXt.y,
            -raXt.z,
            vecB.x,
            vecB.y,
            vecB.z,
            rbXt.x,
            rbXt.y,
            rbXt.z,
        };
        vc = Vector3.Dot(
            -this.unsolvedLinearV
            - Vector3.Cross(this.unsolvedAngularV, vecB)
            + target.unsolvedLinearV
            + Vector3.Cross(target.unsolvedAngularV, vecB)
        , vecB);
        b2 = -(1 - SimplePhysicSolver._Cf) * Math.Sign(vc) * Mathf.Max(Math.Abs(vc) - SimplePhysicSolver._SlopR, 0);

        constrain.SetParameter(p, b2, this.rigidIndex, target.rigidIndex, 2);
    }

    public void ApplyUnsolvedVelocity()
    {
        var deltPositon = this.unsolvedLinearV * Time.fixedDeltaTime;
        var deltRotation = this.unsolvedAngularV * Time.fixedDeltaTime;
        this.unsolvedPosition = this.transform.position + deltPositon;
        var rotated = Quaternion.AngleAxis(deltRotation.magnitude * Mathf.Rad2Deg, Vector3.Normalize(deltRotation)) * this.transform.rotation;
        this.unsolvedRotation = Matrix4x4.TRS(Vector3.zero, rotated, Vector3.one);
    }

    public void ApplySimulation()
    {
        this.linearVelocity = this.unsolvedLinearV;
        var deltPositon = this.linearVelocity * Time.fixedDeltaTime;
        this.angularVelocity = this.unsolvedAngularV;
        var deltRotation = this.angularVelocity * Time.fixedDeltaTime;
        this.transform.position += deltPositon;
        this.transform.rotation = Quaternion.AngleAxis(deltRotation.magnitude * Mathf.Rad2Deg, Vector3.Normalize(deltRotation)) * this.transform.rotation;

        if (deltRotation.magnitude > 0)
        {
            int aa = 0;
        }
        historyPos = (historyPos * historyCount + deltPositon) / (float)(historyCount + 1);
        historyRot = (historyRot * historyCount + deltRotation.magnitude * Mathf.Rad2Deg) / (float)(historyCount + 1);
        historyCount++;
        historyCount = Math.Min(5, historyCount);
        var posChanged = historyPos.magnitude > 0.02 || historyCount < 5;
        var rotChanged = historyRot > 0.01 || historyCount < 5;
        if (historyRot != 0)
        {
            //Debug.Log($"hisRot{historyRot * 100} deltRot{deltRotation.magnitude * Mathf.Rad2Deg * 100} count{historyCount}");
        }
        if (!ground)
        {
            //Debug.Log($"hisPos{historyPos} deltPos{deltPositon} hisRot{historyRot * 100} deltRot{deltRotation.magnitude * Mathf.Rad2Deg * 100} count{historyCount} {posChanged} {rotChanged}");
        }
        if (posChanged)
        {
        }
        if (rotChanged)
        {
        }

        if(!posChanged && !rotChanged)
        {
            //this.collider = false;
            //this.applyCollision = false;
        }
    }

    public float GetP(int i)
    {
        var item = ((i % 6) < 3) ? this.transform.position : this.transform.rotation.eulerAngles;
        return item[i % 3];
    }

    public void DetectCollision(SimplePhysicRigidBody rigid)
    {
        // GJK and EPA method
        // Use GJK to detect Collision;
        var dir = this.unsolvedLinearV - rigid.unsolvedLinearV;
        if (dir.magnitude == 0)
        {
            dir = new Vector3(0, -1, 0);
        }
        //if (res.collision)
        //{
        //    int aaa = 1;
        //}

        if(DebugSimplexEnabled)
        {
            if (debugSimplex == null)
            {
                debugSimplex = new Simplex(this.SupportFunction, rigid.SupportFunction);
                StartCoroutine(this.DebugStep(dir));
            }
            return;
        }

        Simplex simplex = new Simplex(this.SupportFunction, rigid.SupportFunction);
        simplex.Resolve(dir);

        //Debug.Log($"simplex state{simplex.state} insert{simplex.insert} {Time.fixedDeltaTime}");
        CollisionResult collision = new CollisionResult();
        var state = simplex.state;
        collision.collision = false;
        if(state == Simplex.SimplexState.inSimplex
            && simplex.insert > 0)
        {
            collisionGizmosNearest = simplex.normal.normalized * simplex.insert;
            collisionGizmosContact.Clear();
            collisionGizmosContact.Add(simplex.contactA);
            collisionGizmosContact.Add(simplex.contactB);
        }

        if (state == Simplex.SimplexState.inSimplex
            && simplex.insert > 0
            && this.applyCollision 
            && rigid.applyCollision)
        {
            collision.collision = true;
            collision.normalDirection = simplex.normal.normalized;
            collision.contactDirA = simplex.contactA - this.unsolvedPosition;
            collision.contactDirB = simplex.contactB - rigid.unsolvedPosition;
            collision.insertion = simplex.insert;
            if (!this.collisionConstrains.TryGetValue(rigid.rigidIndex, out var constrain))
            {
                constrain = new SimplePhysicConstrainCollision();
                this.collisionConstrains.Add(rigid.rigidIndex, constrain);
            }
            this.AppyCollision(collision, rigid, constrain);
        }
        else
        {
            if (this.collisionConstrains.ContainsKey(rigid.rigidIndex))
            {
                this.collisionConstrains.Remove(rigid.rigidIndex);
            }
        }
        collisionGizmosSimplex = simplex.cornors;
    }

    public IEnumerator DebugStep(Vector3 dir)
    {
        var step = -1;
        while (step != debugSimplex.step)
        {
            yield return new WaitUntil(() => debugImpulse);
            step = debugSimplex.step;
            debugSimplex.Resolve(dir, true);
            collisionGizmosNearest = debugSimplex.normal.normalized;
            collisionGizmosSimplex = debugSimplex.cornors;
            debugImpulse = false;
            Debug.Log($"step {step} state{debugSimplex.state} point Count {debugSimplex.cornors.Count} nomal{debugSimplex.normal} point{debugSimplex.GetNearestPoint()}");
        }
        Debug.Log("finish Step");
        debugSimplex = null;
        yield return null;
    }

    private void OnDrawGizmos()
    {
        Gizmos.color = Color.green;
        for(int i = 0; i < collisionGizmosContact.Count; i += 2)
        {
            Gizmos.DrawSphere(collisionGizmosContact[i], 0.4f);
            Gizmos.DrawSphere(collisionGizmosContact[i + 1], 0.4f);
        }
        Gizmos.DrawLine(this.transform.position, this.transform.position + this.angularVelocity);
        Gizmos.color = Color.red;
        foreach (var pos in collisionGizmosSimplex)
        {
            Gizmos.DrawSphere(pos, 0.2f);
        }
        Gizmos.color = Color.cyan;
        Gizmos.DrawLine(this.transform.position, this.transform.position + collisionGizmosNearest * 10);
    }

    private void OnDrawGizmosSelected()
    {
        if(debugSupport)
        {
            Gizmos.color = Color.green;
            for (int i = 0; i < debugSupportCount; i++)
            {
                for (int j = 0; j < debugSupportCount; j ++)
                {
                    var alpha = (float)i * Math.PI * 2 / (float)debugSupportCount;
                    var theta = (float)j * Math.PI / (float)debugSupportCount;
                    Vector3 dir = new Vector3((float)Math.Cos(alpha), (float)Math.Sin(alpha), 0);
                    dir = dir.normalized;
                    dir.z = (float)Math.Cos(theta);
                    var pos = this.SupportFunction(ref dir);
                    if(supportTarget != null);
                    {
                        dir *= -1;
                        pos = pos - supportTarget.SupportFunction(ref dir);
                    }
                    Gizmos.DrawSphere(pos, 0.1f);
                }
            }
        }
    }
}
