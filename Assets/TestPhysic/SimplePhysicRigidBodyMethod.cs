using System;
using System.Collections;
using System.Collections.Generic;
using UnityEngine;
using static SimplePhysicRigidBody;
using Mathf = UnityEngine.Mathf;
public static class SimplePhysicRigidBodyMethod
{
    public delegate void RigidbodyCollisonMethod(SimplePhysicRigidBody rigidA, SimplePhysicRigidBody rigidB, out CollisionResult result);
    public static RigidbodyCollisonMethod[,] collisionMethods = new RigidbodyCollisonMethod[2, 2]{
            { BoxBoxCollison    ,   BoxPlaneCollison} ,
            { null              ,   PlanePlaneCollison}
        };

    public static void BoxBoxCollison(SimplePhysicRigidBody rigidA, SimplePhysicRigidBody rigidB, out CollisionResult result)
    {
        result = new CollisionResult();
    }

    public static void BoxPlaneCollison(SimplePhysicRigidBody rigidA, SimplePhysicRigidBody rigidB, out CollisionResult result)
    {
        result = new CollisionResult();
        result.collision = false;
        result.normalDirection = Vector3.Normalize(-rigidB.transform.up);
        result.contactDirB = Vector3.Normalize(rigidB.transform.up) * 0.001f;
        var sizeBox = rigidA.transform.localScale;
        var distance = Vector3.Dot(rigidB.transform.position - rigidA.transform.position, result.normalDirection);
        result.insertion = 0;
        if (Math.Abs(Vector3.Dot(rigidA.transform.up, rigidB.transform.up)) == 1 && sizeBox.y / 2 > distance)
        {
            result.insertion = sizeBox.y / 2 - distance;
            result.collision = true;
            result.contactDirA = (Vector3.Dot(rigidA.transform.up, -rigidB.transform.up) > 0 ? 1 : -1) * rigidA.transform.up;
            result.contactDirA = Vector3.Normalize(result.contactDirA) * sizeBox.y / 2;
            return;
        }
        if (Math.Abs(Vector3.Dot(rigidA.transform.forward, rigidB.transform.up)) == 1 && sizeBox.z / 2 > distance)
        {
            result.insertion = sizeBox.z / 2 - distance;
            result.collision = true;
            result.contactDirA = (Vector3.Dot(rigidA.transform.forward, -rigidB.transform.up) > 0 ? 1 : -1) * rigidA.transform.forward;
            result.contactDirA = Vector3.Normalize(result.contactDirA) * sizeBox.z / 2;
            return;
        }
        if (Math.Abs(Vector3.Dot(rigidA.transform.right, rigidB.transform.up)) == 1 && sizeBox.x / 2 > distance)
        {
            result.insertion = sizeBox.x / 2 - distance;
            result.collision = true;
            result.contactDirA = (Vector3.Dot(rigidA.transform.right, -rigidB.transform.up) > 0 ? 1 : -1) * rigidA.transform.right;
            result.contactDirA = Vector3.Normalize(result.contactDirA) * sizeBox.x / 2;
            return;
        }
        var cornor = Vector3.zero;
        var boxMatrix = rigidA.unsolvedRotation;
        List<Vector3> points = new List<Vector3>();
        for (int i = 0; i < 8; ++i)
        {
            cornor.Set(
                ZeroOneToMinusOneOne((i / 4) % 2) * sizeBox.x / 2,
                ZeroOneToMinusOneOne((i / 2) % 2) * sizeBox.y / 2,
                ZeroOneToMinusOneOne(i % 2) * sizeBox.z / 2);
            cornor = boxMatrix.MultiplyPoint(cornor);
            //var d = Vector3.Distance(cornor, Vector3.zero);
            var cornorDistance = Vector3.Dot(cornor, result.normalDirection);
            if (cornorDistance > distance)
            {
                points.Add(cornor);
                result.insertion = Math.Max(result.insertion, cornorDistance - distance);
            }
        }

        int count = 0;
        if (points.Count > 0)
        {
            Vector3 point = new Vector3();
            for (int i = 0; i < points.Count; ++i)
            {
                var cornorDistance = Vector3.Dot(points[i], result.normalDirection);
                if (cornorDistance - distance > result.insertion - 1e-5)
                {
                    point += points[i];
                    count++;
                }
            }

            point /= count == 0 ? 1 : count;
            result.contactDirA = point;
            //Debug.Log($"Collision Point {point} length{point.magnitude} insert{result.insertion}");
            result.collision = true;
        }
        //float vc = Vector3.Dot(
        //    -rigidA.unsolvedLinearV
        //    - Vector3.Cross(rigidA.unsolvedAngularV, result.contactDirA)
        //    + rigidB.unsolvedLinearV
        //    + Vector3.Cross(rigidB.unsolvedAngularV, result.contactDirB)
        //, result.normalDirection);
        //var wv = Vector3.Dot(-rigidA.unsolvedLinearV, result.normalDirection);
        //var wa = Vector3.Dot(-Vector3.Cross(rigidA.unsolvedAngularV, result.contactDirA), result.normalDirection);
        //if (vc > 0 && result.collision)
        //{
        //    //Debug.Log($"collision while departing wv={wv} wa={wa}");
        //    int a = 0;
        //}
        return;
    }

    private static int ZeroOneToMinusOneOne(int i)
    {
        return 2 * i - 1;
    }

    public static void PlanePlaneCollison(SimplePhysicRigidBody rigidA, SimplePhysicRigidBody rigidB, out CollisionResult result)
    {
        result = new CollisionResult();
    }

}
