using System.Collections;
using System.Collections.Generic;
using UnityEngine;

public class FieldOfView : MonoBehaviour
{
    #region Variables
    public float viewRadius;
    [Range(0, 360)]
    public float viewAngle;

    public LayerMask targetMask;
    public LayerMask Unwalkable;

    [HideInInspector]
    public List<Transform> visibleTargets = new List<Transform>();

    public float meshResolution;
    public int EdgeResolveIterations;
    public float edgeDistThreshold;

    public MeshFilter viewMeshFilter;
    Mesh viewMesh;
    #endregion

    // Start is called before the first frame update
    void Start()
    {

        viewMesh = new Mesh();
        viewMesh.name = "View mesh";

        viewMeshFilter.mesh = viewMesh;
        StartCoroutine("FindTargetWithDelay", .2f);
    }



    // Update is called once per frame
    void LateUpdate()
    {
        DrawFOV(); 
    }

    IEnumerator FindTargetWithDelay(float delay)
    {
        while (true)
        {
            yield return new WaitForSeconds(delay);
            FindPlayer();
        }
    }

    //locates the player
    void FindPlayer()
    {
        visibleTargets.Clear();
        Collider[] targetinViewRadius = Physics.OverlapSphere(transform.position, viewRadius, targetMask);

        for (int i = 0; i < targetinViewRadius.Length; i++)
        {
            Transform player = targetinViewRadius [i].transform;
            Vector3 dirToTarget = (player.position - transform.position).normalized;

            if (Vector3.Angle (transform.forward, dirToTarget) < viewAngle / 2)
            {
                float dstToTarget = Vector3.Distance(transform.position, player.position);

                if (!Physics.Raycast(transform.position, dirToTarget, dstToTarget, Unwalkable))
                {
                    visibleTargets.Add(player);
                }
            }
        }
    }

    //draw Field of view mesh
    void DrawFOV()
    {
        int stepCount = Mathf.RoundToInt(viewAngle * meshResolution);
        float stepAngleSize = viewAngle / stepCount;
        List<Vector3> viewPoints = new List<Vector3>();
        ViewCastInfo oldViewCast = new ViewCastInfo();

        for (int i = 0; i < stepCount; i++)
        {
            float angle = transform.eulerAngles.y - viewAngle / 2 + stepAngleSize * i;
            ViewCastInfo newViewCast = ViewCast(angle);

            if (i > 0)
            {
                bool edgeDistThresholdExceeded = Mathf.Abs(oldViewCast.distance - newViewCast.distance) > edgeDistThreshold;
                if (oldViewCast.hit != newViewCast.hit || (oldViewCast.hit && newViewCast.hit && edgeDistThresholdExceeded))
                {
                    EdgeInfo edge = FindEdge(oldViewCast, newViewCast);

                    if (edge.pointA != Vector3.zero)
                    {
                        viewPoints.Add(edge.pointA);
                    }
                    if (edge.pointB != Vector3.zero)
                    {
                        viewPoints.Add(edge.pointB);
                    }
                }
            }
            viewPoints.Add(newViewCast.point);
            oldViewCast = newViewCast;
        }

        int vertexcount = viewPoints.Count + 1;
        Vector3[] vertices = new Vector3[vertexcount];
        int[] triangles = new int[(vertexcount - 2) * 3];

        vertices[0] = Vector3.zero;
        for (int i = 0; i < vertexcount - 1; i++)
        {
            vertices[i + 1] = transform.InverseTransformPoint(viewPoints[i]);

            //make sure the ray doesn't go out of bounds
            if ( i< vertexcount - 2)
            {
            triangles[i * 3] = 0;
            triangles[i * 3 + 1] = i + 1;
            triangles[i * 3 + 2] = i + 2;
            }

            viewMesh.Clear();
            viewMesh.vertices = vertices;
            viewMesh.triangles = triangles;
            viewMesh.RecalculateNormals();
        }
    }

    ViewCastInfo ViewCast( float globalAngle)
    {
        Vector3 direction = DirFromAngle(globalAngle, true);
        RaycastHit hit;

        if(Physics.Raycast(transform.position, direction, out hit, viewRadius, Unwalkable))
        {
            return new ViewCastInfo(true, hit.point, hit.distance, globalAngle);
        }
        else
        {
            return new ViewCastInfo(false, transform.position + direction * viewRadius, viewRadius, globalAngle);
        }
    }

    //how far field of view goes in an angle
    public Vector3 DirFromAngle(float angleInDegrees, bool angleIsGlobal)
    {
        if (!angleIsGlobal)
        {
            angleInDegrees += transform.eulerAngles.y;
        }
        return new Vector3(Mathf.Sin(angleInDegrees * Mathf.Deg2Rad), 0, Mathf.Cos(angleInDegrees * Mathf.Deg2Rad));
    }

    public struct ViewCastInfo
    {
        public bool hit;
        public Vector3 point;
        public float distance;
        public float angle;

        public ViewCastInfo(bool _hit, Vector3 _point, float _distance, float _angle)
        {
            hit = _hit;
            point = _point;
            distance = _distance;
            angle = _angle;
        }
    }

    public struct EdgeInfo
    {
        public Vector3 pointA;
        public Vector3 pointB;

        public EdgeInfo(Vector3 _PointA, Vector3 _PointB)
        {
            pointA = _PointA;
            pointB = _PointB;
        }
    }

    EdgeInfo FindEdge(ViewCastInfo MinviewCast, ViewCastInfo MaxviewCast)
    {
        float minAngle = MinviewCast.angle;
        float maxAngle = MaxviewCast.angle;
        Vector3 minPoint = Vector3.zero;
        Vector3 maxPoint = Vector3.zero;

        //cast ray for min and max angles of the Field of view

        for (int i = 0; i < EdgeResolveIterations; i++)
        {
            float angle = (minAngle + maxAngle) / 2;
            ViewCastInfo newViewCast = ViewCast(angle);

            bool edgeDistThresholdExceeded = Mathf.Abs(MinviewCast.distance - newViewCast.distance) > edgeDistThreshold;
            if (newViewCast.hit == MinviewCast.hit && !edgeDistThresholdExceeded)
            {
                minAngle = angle;
                minPoint = newViewCast.point;
            }
            else
            {
                maxAngle = angle;
                maxPoint = newViewCast.point;
            }
        }

        return new EdgeInfo(minPoint, maxPoint);
    }

}
