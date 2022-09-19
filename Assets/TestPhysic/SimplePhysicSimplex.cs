using System;
using System.Collections;
using System.Collections.Generic;
using UnityEngine;
using Mathf = UnityEngine.Mathf;

public class Simplex
{
    private const int maxLoop = 10;
    private const int epaCloserMin = 4;
    public SimplexState state { get; private set; }
    public delegate Vector3 SupportFunction(ref Vector3 dir);
    public List<Vector3> cornors { get; private set; } = new List<Vector3>();
    public Vector3 contactA;
    public Vector3 contactB;
    public Vector3 normal;
    public float insert;
    private Vector3 nearest;
    int scale = 0;
    List<Vector3Int> triangles = new List<Vector3Int>();
    // x,y,z as direction, w as length;
    List<Vector3> triangleNearest = new List<Vector3>();
    List<Vector3> triangleNormals = new List<Vector3>();
    private SupportFunction supportA;
    private SupportFunction supportB;
    private Dictionary<Vector2Int, int> edgesTobeSeamed = new Dictionary<Vector2Int, int>();
    public int step;
    private int GDKCount = 0;
    private float epaInsertCache = float.MaxValue;
    private int epaCloserCount = 0;

    public enum SimplexState
    {
        unsolved,
        inSimplex,
        outSimplex,
    }

    public Vector3 GetNearestPoint()
    {
        return nearest;
    }

    public Simplex(SupportFunction sA, SupportFunction sB)
    {
        supportA = sA;
        supportB = sB;
        state = SimplexState.unsolved;
    }

    public void Resolve(Vector3 dir, bool debug = false)
    {
        Vector3 pos;
        if (step == 0)
        {
            pos = Support(ref dir);
            this.AddPointGJK(pos, dir);
            step++;
            if (debug)
                return;
        }

        while (state == Simplex.SimplexState.unsolved && step < maxLoop)
        {
            dir = (scale == 1 ? -nearest : normal);
            if (dir.magnitude == 0)
            {
                state = SimplexState.inSimplex;
                return;
            }
            pos = Support(ref dir);
            this.AddPointGJK(pos, dir);
            step++;
            GDKCount++;
            if (debug)
            {
                return;
            }
        }
        if (step == maxLoop)
        {
            state = SimplexState.outSimplex;
            //Debug.Log($"simplex reach max loop GDK{GDKCount} {step}");
            return;
        }
        if (state == SimplexState.inSimplex)
        {
            while (step < maxLoop + GDKCount)
            {
                if (TouchNearstPointOfSimplex(out pos))
                {
                    break;
                }
                ReformTriangles(pos);
                step++;
                if (debug)
                {
                    return;
                }
            }
        }

        if (step == maxLoop + GDKCount)
        {
            // Some testing for very close touch.
            // Some sphere collider may reach epa loop limit and always reaching nearest.
            if(epaCloserCount < epaCloserMin || insert > 0.5f)
            {
                state = SimplexState.outSimplex;
            }
            //Debug.Log($"simplex reach max loop EPA{GDKCount} {step} insert{insert} {epaCloserCount}");
            return;
        }
    }

    private Vector3 Support(ref Vector3 dir)
    {
        contactA = supportA(ref dir);
        dir *= -1;
        contactB = supportB(ref dir);
        dir *= -1;
        return contactA - contactB;
    }

    private bool TouchNearstPointOfSimplex(out Vector3 pos)
    {
        pos = Vector3.zero;
        float newNearDis = float.MaxValue;

        for (int i = 0; i < triangles.Count; ++i)
        {
            var tNear = triangleNearest[i];
            var tNomal = triangleNormals[i];
            var dis = Vector3.Dot(tNear, -tNomal);
            if (dis <= newNearDis)
            {
                newNearDis = dis;
                //newNearDis = triangleNormals[i];
                // nearest = triangleNearest[i];
                // Check all those zero normals, if all touch edge, shall stop looping.
                if (dis == 0)
                {
                    var refTn = tNomal;
                    //NearestPointOfTriangle(cornors[triID.x], cornors[triID.y], cornors[triID.z], out var p, out var n, out var _);
                    var trySearch = Support(ref refTn);
                    var search1 = SideSearched(trySearch, tNomal, out var sideDis);
                    if (search1)
                    {
                        if (sideDis == 0)
                        {
                            insert = sideDis;
                            normal = tNomal;
                            return true;
                        }
                        if (sideDis < newNearDis)
                        {
                            newNearDis = sideDis;
                            normal = tNomal;
                        }
                    }
                    refTn *= -1;
                    trySearch = Support(ref refTn);
                    var search2 = SideSearched(trySearch, -tNomal, out sideDis);
                    if (search2)
                    {
                        if (sideDis == 0)
                        {
                            insert = sideDis;
                            normal = tNomal;
                            return true;
                        }
                        if (sideDis < newNearDis)
                        {
                            newNearDis = sideDis;
                            normal = -tNomal;
                        }
                    }
                    if (search1 && search2)
                    {
                        // may not happen, but we will delete the triangle.
                        Debug.Log("Wrong plane, double side extended");
                        triangles.RemoveAt(i);
                        triangleNearest.RemoveAt(i);
                        triangleNormals.RemoveAt(i);
                        i--;
                    }
                    else
                    {
                        normal = search1 ? -tNomal : tNomal;
                    }
                }
                else
                {
                    normal = -tNomal;
                    //normal = nearest;
                }
                nearest = normal * tNear.magnitude;
            }
        }

        var refN = normal;
        pos = Support(ref refN);
        return TouchNearstPointOfSimplex(pos, normal);

        //// If root on the edge and reach edge, check the optical direction of nomal;
        //// but the near position is used as final result;
        //if (res1 && nearest.magnitude == 0)
        //{
        //    var cA = contactA;
        //    var cB = contactB;
        //    var insertCache = insert;
        //    var posCache = pos;
        //    dir *= -1;
        //    pos = Support(ref dir);
        //    var res2 = TouchNearstPointOfSimplex(pos, dir);
        //    if (res2 && insertCache < insert)
        //    {
        //        contactA = cA;
        //        contactB = cB;
        //        dir *= -1;
        //        insert = Math.Min(insertCache, insert);
        //        pos = posCache;
        //    }
        //    return res2;
        //}
    }

    private bool TouchNearstPointOfSimplex(Vector3 pos, Vector3 dir)
    {
        // Alway Expand simplex by new postion, except when the position is no further;
        float near = Vector3.Dot(dir, pos);
        float currentNear = Vector3.Dot(nearest, dir);
        insert = near;
        //Debug.Log($"near {near} = {dir}{pos} nearest{currentNear}{nearest}{dir}");
        var dis = near - currentNear;
        if(insert < epaInsertCache + SimplePhysicSolver._Epsilon)
        {
            epaCloserCount++;
        }
        else
        {
            epaCloserCount = 0;
        }

        //Debug.Log($"insert change: {insert}  cache {epaInsertCache}");
        epaInsertCache = insert;
        if (near < SimplePhysicSolver._Epsilon || dis <= SimplePhysicSolver._Epsilon)
        {
            return true;
        }

        return false;
    }

    private bool SideSearched(Vector3 pos, Vector3 dir, out float _insert)
    {
        // Alway Expand simplex by new postion, except when the position is no further;
        float near = Vector3.Dot(dir, pos);
        if(near < 0)
        {
            Debug.Log($"wrong support point");
        }
        _insert = near;
        if (near < SimplePhysicSolver._Epsilon)
        {
            return true;
        }

        for (int i = 0; i < scale; ++i)
        {
            var dis = Vector3.Dot(dir, cornors[i]);
            if (dis >= near)
            {
                return true;
            }
        }

        return false;
    }

    private void ReformTriangles(Vector3 pos)
    {
        switch (scale)
        {
        case 1:
            AddPointDirectly(pos);
            break;
        case 2:
            {
                AddPointDirectly(pos);
                this.NearestPointOfTriangle(cornors[0], cornors[1], cornors[2], out var near, out var n, out var _);
                nearest = near;
                normal = n;
                break;
            }
        case 3:
            {
                AddPointDirectly(pos);
                for (int i = 0; i < this.scale; ++i)
                {
                    for (int j = i + 1; j < this.scale; ++j)
                    {
                        for (int k = j + 1; k < this.scale; ++k)
                        {
                            triangles.Add(new Vector3Int(i, j, k));
                            // skip one un necessary calculate.
                            if (k == 2)
                            {
                                triangleNearest.Add(nearest);
                                triangleNormals.Add(normal);
                                var dis = Vector3.Dot(nearest, normal);
                                //triangleNearest.Add(new Vector4(normal.x, normal.y, normal.z, Math.Abs(Vector3.Dot(nearest, normal))));
                            }
                            else
                            {
                                NearestPointOfTriangle(cornors[i], cornors[j], cornors[k], out var near, out var n, out var _);
                                triangleNearest.Add(near);
                                triangleNormals.Add(n);
                                //triangleNearest.Add(new Vector4(n.x, n.y, n.z, Math.Abs(Vector3.Dot(near, n))));
                                //// here use <= ,when several distance == 0;make all of them looped
                                //if (near.magnitude < nearest.magnitude)
                                //{
                                //    if (near.magnitude == 0)
                                //    {
                                //        var trySearch = Support(ref n);
                                //        if(!TouchNearstPointOfSimplex(trySearch, n))
                                //        {
                                //            nearest = near;
                                //            normal = n;
                                //        }
                                //    }
                                //    else
                                //    {
                                //        nearest = near;
                                //        normal = near;
                                //    }
                                //}
                            }
                        }
                    }
                }
                break;
            }
        default:
            // scale >= 4
            Vector3 center = Vector3.zero;
            for (int i = 0; i < cornors.Count; ++i)
            {
                center += cornors[i];
            }
            // Although some cornors is dummy, the average still inside simplex any way;
            center /= cornors.Count;

            for (int i = 0; i < triangles.Count; ++i)
            {
                var tri = triangles[i];
                PointSideOfTriangle(cornors[tri.x], cornors[tri.y], cornors[tri.z], pos, center, out bool sameSide, out bool onPlane);
                if (onPlane)
                {
                    for (int j = 0; j < cornors.Count; ++j)
                    {

                    }
                    sameSide &= true;
                }
                if (!sameSide)
                {
                    triangles.RemoveAt(i);
                    triangleNearest.RemoveAt(i);
                    triangleNormals.RemoveAt(i);
                    --i;
                    for (int j = 0; j < 3; ++j)
                    {
                        int x = tri[j % 3];
                        int y = tri[(j + 1) % 3];
                        Vector2Int edge = new Vector2Int(x < y ? x : y, x < y ? y : x);
                        if (edgesTobeSeamed.ContainsKey(edge))
                        {
                            edgesTobeSeamed[edge] += 1;
                        }
                        else
                        {
                            edgesTobeSeamed.Add(edge, 1);
                        }
                    }
                }
            }

            var posID = scale;
            AddPointDirectly(pos);
            foreach (var edge in edgesTobeSeamed)
            {
                if (edge.Value == 1)
                {
                    NearestPointOfTriangle(cornors[edge.Key.x], cornors[edge.Key.y], cornors[posID], out var near, out var n, out var _);
                    // N = ONE when pos is on the same position as existing cornors.
                    if(n.magnitude != 0)
                    {
                        triangles.Add(new Vector3Int(edge.Key.x, edge.Key.y, posID));
                        //triangleNearest.Add(new Vector4(n.x, n.y, n.z, Math.Abs(Vector3.Dot(p, n ))));
                        triangleNearest.Add(near);
                        triangleNormals.Add(n);
                    }
                }
            }

            edgesTobeSeamed.Clear();
            break;
        }
    }

    private void ExpandSimplexGJK(Vector3 pos, Vector3 dir)
    {
        // If simplex scale == 4, force to replace the farest Point.
        // And check if the new point is at the same dimension of the origin simplex, like on the point or line....
        // (think twice! now way a new point is inside the simplex, because new point is at he farest position of the direction)
        // (it happens only when the face or the line is vertically nearest to the root, and the point is the nearest point）
        // (in this case, several points will at the same distance as the new point for given direction, whitch means loop ends).
        int index = 0;
        float near = Vector3.Dot(dir, pos);
        float currentNear = Vector3.Dot(dir, nearest);
        if (scale > 0 && near <= currentNear + SimplePhysicSolver._Epsilon)
        {
            state = SimplexState.outSimplex;
            return;
        }
        for (int i = 0; i < scale; ++i)
        {
            if((cornors[i] - pos).magnitude == 0)
            {
                state = SimplexState.outSimplex;
                return;
            }
            var dis = Vector3.Dot(dir, cornors[i]);
            if (dis < near)
            {
                index = i;
                near = dis;
            }
        }
        if (scale == 4)
        {
            cornors[index] = pos;
            return;
        }
        AddPointDirectly(pos);
    }


    private void CheckInsideAndNearestGJK()
    {
        if (state != SimplexState.unsolved)
            return;
        // new point should be at the same phase with simplex, for example, if simpex is a triangle, point should on the same plane;
        // A Cross method will reduce one phase difference. 
        switch (scale)
        {
        case 1:
            {
                if (cornors[0] == Vector3.zero)
                {
                    state = SimplexState.inSimplex;
                    normal = Vector3.up;
                }
                else
                {
                    normal = -cornors[0];
                }
                nearest = cornors[0];
            }
            break;
        case 2:
            NearestPointOfLine(cornors[0], cornors[1], out nearest, out normal, out var inLine);
            if (inLine)
            {
                state = SimplexState.inSimplex;
            }
            break;
        case 3:
            NearestPointOfTriangle(cornors[0], cornors[1], cornors[2], out nearest, out normal, out var inTriangle);
            if (inTriangle)
            {
                state = SimplexState.inSimplex;
            }
            break;
        case 4:
            NearestPointOfTetrahedron(cornors[0], cornors[1], cornors[2], cornors[3], out nearest, out normal, out var inTetra);
            if (inTetra)
            {
                state = SimplexState.inSimplex;
            }
            break;
        default:
            state = SimplexState.inSimplex;
            break;
        }
    }

    private void AddPointGJK(Vector3 pos, Vector3 dir)
    {
        // Basic steps:
        // Step 1 : deal with the new point and simplex.
        // Step 2 : theck if the root point is in the simplex, if so collision Check loop will be ended.
        // Step 3 : calculate nearst point for future use, this will be contained in step 2 for sharing some calculation.
        ExpandSimplexGJK(pos, dir);
        CheckInsideAndNearestGJK();
    }

    private Vector3 GetNearPoint()
    {
        return nearest;
    }

    private void AddPointDirectly(Vector3 pos)
    {
        cornors.Add(pos);
        scale++;
    }

    private void PointSideOfLine(Vector3 a, Vector3 b, Vector3 c, out bool side)
    {
        var ab = b - a;
        var ac = c - a;
        var ao = -a;
        var cSide = Vector3.Cross(ab, ac);
        var oSide = Vector3.Cross(ab, ao);
        side = Vector3.Dot(cSide, oSide) >= 0;
    }

    private void NearestPointOfLine(Vector3 a, Vector3 b, Vector3 target, out Vector3 p, out Vector3 normal, out bool inLine)
    {
        NearestPointOfLine(a - target, b - target, out var biasedP, out normal, out inLine);
        p = biasedP + target;
    }

    private void NearestPointOfLine(Vector3 a, Vector3 b, out Vector3 p, out Vector3 n, out bool inLine)
    {
        inLine = false;
        var dot = Vector3.Dot(a.normalized, b.normalized);
        if (-(dot) == 1)
        {
            inLine = true;
        }

        var ab = b - a;
        var ao = -a;
        var bias = Vector3.Dot(ab.normalized, ao);
        bias = Math.Clamp(bias, 0, ab.magnitude);
        p = a + bias * ab.normalized;

        var up = Vector3.up;
        var sameLine = Vector3.Dot(up, ab.normalized);
        if (Math.Abs(sameLine) == 1)
        {
            up = Vector3.right;
        }
        n = Vector3.Cross(up, ab).normalized;
        var nXao = Vector3.Dot(n, -a.normalized);
        n = n * (Math.Sign(nXao) > 0 ? 1 : -1);
        return;
        //if (inLine)
        //{
        //    var up = Vector3.up;
        //    if (Vector3.Dot(up, (a - b).normalized) == 1)
        //    {
        //        up = Vector3.right;
        //    }
        //    n = Vector3.Cross(up, a - b).normalized;
        //}
        //else
        //{
        //    n = p;
        //}
    }

    private void PointSideOfTriangle(Vector3 a, Vector3 b, Vector3 c, Vector3 d, Vector3 target, out bool side, out bool OnPlane)
    {
        PointSideOfTriangle(a - target, b - target, c - target, d - target, out side, out OnPlane);
    }

    private void PointSideOfTriangle(Vector3 a, Vector3 b, Vector3 c, Vector3 d, out bool side, out bool OnPlane)
    {
        var ab = (b - a);
        var ac = (c - a);
        var n = Vector3.Cross(ab, ac);
        var nDad = Vector3.Dot(n, d - a);
        var nDao = Vector3.Dot(n, -a);
        side = nDad * nDao >= 0;
        OnPlane = nDad * nDao == 0;
    }

    private void NearestPointOfTriangle(Vector3 a, Vector3 b, Vector3 c, out Vector3 p, out Vector3 n, out bool inTriangle)
    {
        var ab = (b - a);
        var ac = (c - a);

        // Then if the root Pos is on the Triangle plane, should check if the root is in the simplex.
        // if not, find projected point on the plane.
        n = Vector3.Cross(ab, ac).normalized;
        var nXao = Vector3.Dot(n, -a.normalized);
        n = n * (Math.Sign(nXao) > 0 ? 1 : -1);
        var inPlane = Math.Abs(nXao) < SimplePhysicSolver._Epsilon;
        var bias = Vector3.Dot(n, -a);
        var projected = (-n) * bias;
        int insideCount = 0;
        float dis = int.MaxValue;

        p = projected;
        // try find the nearest point of projected point to edgeLine
        // if projected point is not in the triangle ,then it is the nearest Point.
        for (int i = 0; i < 3; i++)
        {
            LoopNext(ref a, ref b, ref c);
            PointSideOfLine(a, b, c, out var side);
            insideCount += side ? 1 : 0;
            NearestPointOfLine(a, b, projected, out var near, out var _n, out var _);
            var nearDis = (near - projected).magnitude;
            if (nearDis < dis)
            {
                dis = nearDis;
                p = near;
            }
        }

        bool projectInTriangle = insideCount == 3;
        inTriangle = (inPlane && projectInTriangle);
        if (projectInTriangle)
        {
            p = projected;
        }
        //else
        //{
        //    n = p;
        //}
    }

    private void NearestPointOfTetrahedron(Vector3 a, Vector3 b, Vector3 c, Vector3 d, out Vector3 p, out Vector3 n, out bool inTetrahedron)
    {
        int insideCount = 0;
        float dis = int.MaxValue;
        p = nearest;
        n = normal;
        int skipIndex = 3;
        this.triangles.Clear();
        this.triangleNearest.Clear();
        this.triangleNormals.Clear();
        inTetrahedron = false;
        for (int i = 0; i < 4; ++i)
        {
            PointSideOfTriangle(a, b, c, d, out var side, out var _);
            insideCount += side ? 1 : 0;
            NearestPointOfTriangle(a, b, c, out var near, out var _n, out var inTri);

            if(inTri)
            {
                inTetrahedron = true;
            }
            var nearDis = near.magnitude;
            if (nearDis < dis)
            {
                dis = nearDis;
                p = near;
                n = _n;
            }
            LoopNext(ref a, ref b, ref c, ref d);
            skipIndex = (skipIndex + 1) % 4;
            this.triangles.Add(new Vector3Int(skipIndex, (skipIndex + 1) % 4, (skipIndex + 2) % 4));
            //this.triangleNearest.Add(new Vector4(_n.x, _n.y, _n.z, near.magnitude));
            triangleNearest.Add(near);
            triangleNormals.Add(_n);
        }

        inTetrahedron |= insideCount == 4;
    }

    private void LoopNext(ref Vector3 a, ref Vector3 b, ref Vector3 c)
    {
        Vector3 t = a;
        a = b;
        b = c;
        c = t;
    }

    private int TriangleHash()
    {
        return 0;
    }

    private void LoopNext(ref Vector3 a, ref Vector3 b, ref Vector3 c, ref Vector3 d)
    {
        Vector3 t = a;
        a = b;
        b = c;
        c = d;
        d = t;
    }

}
