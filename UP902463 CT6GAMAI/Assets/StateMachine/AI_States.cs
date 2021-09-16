using System.Collections;
using System.Collections.Generic;
using UnityEngine;
using UnityEngine.AI;

//AI states 
public enum BehaviourStates
{
    none,
    Wander,
    Patrol,
    Chase
}


public class AI_States : MonoBehaviour
{
    public BehaviourStates InitialState; //variable for starting state of AI

    #region (State variables)
    //wander state variables
    [Header("Wander Settings")]
    public Bounds boundbox; //bounding box for telling the AI where to walk

    //patrol state variables
    [Header("Patrol Settings")]
    public Transform[] PatrolPoints; //array of patrol points
    public bool RandomSequence = false; //go to random point

    //chase state variables
    [Header("Chase Settings")]
    public float ChaseDistance = 2.5f; //distance of how far the AI will chase the player
    public Transform player;

    private NavMeshAgent agent;
    private BehaviourStates currentState = BehaviourStates.none;
    private Vector3 targetPos; //target position for where the Ai walks to
    #endregion

    #region SteeringBehaviours
    [Header("Steering Behaviours Settings")]
    public Vector3 Velocity;

    //Position, Heading and Side can be accessed from the transform component with transform.position, transform.forward and transform.right respectively

    //Represents the weight of an object, will effect its acceleration
    public float Mass = 1;

    //The maximum speed this agent can move per second
    public float MaxSpeed = 1;

    //The thrust this agent can produce
    public float MaxForce = 1;
    #endregion

    #region Perception Variables
    [Header("Perception Settings")]
    public float viewRadius; //radius of the FOV
    [Range(0, 360)]
    public float viewAngle; //Angle of the FOV

    public LayerMask targetMask; //detect the player within this mask
    public LayerMask Unwalkable; //layer for the walls

    [HideInInspector]
    public List<Transform> visibleTargets = new List<Transform>(); //array of targets

    public float meshResolution; //resolution for FOV mesh 
    public int EdgeResolveIterations;
    public float edgeDistThreshold;

    public MeshFilter viewMeshFilter; //material for mesh filter
    Mesh viewMesh; //mesh for FOV
    #endregion


    private void Awake()
    {
        agent = GetComponent<NavMeshAgent>();       
    }

    // Start is called before the first frame update
    void Start()
    {
        //start with the state that is assigned to be the initial state.
        SetState(InitialState);

        viewMesh = new Mesh();
        viewMesh.name = "View mesh";

        viewMeshFilter.mesh = viewMesh;
        StartCoroutine("FindTargetWithDelay", .2f); //start coroutine
    }

    // Update is called once per frame
    void Update()
    {
        DrawFOV();

        //target detection and chasing
        if (player != null)
        {

            //if the target is less than the chase distance
            if (visibleTargets.Contains(player))
            {
                //if AI is not in it's chase state
                if (currentState != BehaviourStates.Chase)
                {
                    GetComponent<Seek>().SeekEnabled = true; //seek behaviour = true                
                    SetState(BehaviourStates.Chase); //set to chase state
                }
                else
                {
                    GetComponent<Seek>().SeekTargetPos = player.position; //get SeekTargetPosition and make it equal player position
                    //targetPos = player.position;
                    agent.SetDestination(GetComponent<Seek>().SeekTargetPos); // set the destination for AI to walk to
                }
            }
            else if (currentState == BehaviourStates.Chase)
            {
                GetComponent<Seek>().SeekEnabled = false; //seek behaviour = false
                SetState(InitialState);//if target position is outside FOV, return to initial state              
            }
        }

        //stopping distances
        float Distance = Vector3.Distance(targetPos, transform.position);
        
        if (Distance <= agent.stoppingDistance)      
        {
            agent.isStopped = true; //isstopped: holds the stop and resume condition of the NavMeshAgent
             if(currentState == BehaviourStates.Wander)
            {
                FindNewWanderTarget();
            }
             else if (currentState == BehaviourStates.Patrol) //is current state = patrol
            {
                GoToNextPatrolPoint(RandomSequence);
            }

        }
        else if (agent.isStopped == true)
        {
            agent.isStopped = false;
        }
             
        if (currentState == BehaviourStates.Chase) //If current state = chase
            {
                ChasePlayer(); //update physics of AI every frame
            }
    }

    
    // set the current state
    void SetState(BehaviourStates AI)
    {
        if (currentState != AI)
        {
            currentState = AI; //set current state to whatever the state that the AI is defined
            if (currentState == BehaviourStates.Wander)
            {
                //find a new wander state
                FindNewWanderTarget();
            }
            else if (currentState == BehaviourStates.Patrol)
            {
                //Go to patrol point
                GoToNextPatrolPoint(RandomSequence);
            }             
            else if (currentState == BehaviourStates.Chase)
            {
                //targetPos = player.position;
                //agent.SetDestination(player.position);
                //agent.isStopped = true;
                ChasePlayer();

            }
        }
    }

    //chase the player
    void ChasePlayer()
    {
                Vector3 SteeringForce = Vector3.zero;

                SteeringBehaviourBase[] steering = gameObject.GetComponents<SteeringBehaviourBase>();

                foreach (SteeringBehaviourBase i in steering)
                {
                    SteeringForce += i.Calculate();
                }

                Vector3 Acceleration = SteeringForce / Mass; //calculate an accerleration

                Velocity += Acceleration * Time.deltaTime; //update the velocity

                Velocity = Vector3.ClampMagnitude(Velocity, MaxSpeed); //Clamp the magnitude of our velocity to Max Speed

                if (Velocity != Vector3.zero)
                {
                    transform.position += Velocity * Time.deltaTime; //update the position

                    //transform.forward = Velocity.normalized; //update the heading

                    //transform.right 
                }
    }

    #region WanderState functions
    void FindNewWanderTarget()
    {
        //set targetPos to a random location in the bounding box
        targetPos = GetRandomPoint();
        //set the agent destination to targetPos
        agent.SetDestination(targetPos);
        //make sure the agent can move (is not stopped)
        agent.isStopped = false;
    }

    //find a random point in the bound box
    Vector3 GetRandomPoint()
    {
        //find random location in boundbox
        float RandomX = Random.Range(-boundbox.extents.x + agent.radius, boundbox.extents.x - agent.radius); 
        float RandomZ = Random.Range(-boundbox.extents.z + agent.radius, boundbox.extents.z - agent.radius); 

        //return a new Vector3
        //transform.position.y keeps the AI on the same level that the AI is positioned in the world
        return new Vector3(RandomX, transform.position.y, RandomZ);
    }
#endregion

    #region PatrolState functions

    //go to the next patrol point
    void GoToNextPatrolPoint(bool random = false)
    {
        if (random == false) //checks if patrol points are selected at random
        {
            //go to next patrol point in list
            targetPos = GetPatrolPoint();
        }
        else
        {
            //go to random point
            targetPos = GetPatrolPoint(true);
        }
        //set the destination to the new target position
        agent.SetDestination(targetPos);
        //make sure the agent can move(is not stopped)
        agent.isStopped = false;
    }

   
    //find the patrol point in array
    Vector3 GetPatrolPoint(bool random = false)
    {
        if(random == false) //checks if patrol points are selected at random
        {
            if (targetPos == Vector3.zero) // is target in default position?
            {
                return PatrolPoints[0].position; //go to first point in array
            }
            else
            {
                for (int i = 0; i < PatrolPoints.Length; i++)
                {
                    if(PatrolPoints[i].position == targetPos)
                    {
                        if (i + 1 >= PatrolPoints.Length) //has the AI reached last point in Array?
                        {
                            return PatrolPoints[0].position; // return to starting point
                        }
                        else
                        {
                            return PatrolPoints[i + 1].position;
                        }
                    }
                }
                //get closest patrol point
                return GetClosestPatrolPoint();
            }
        }
        else
        {
            return PatrolPoints[Random.Range(0, PatrolPoints.Length)].position; //find a random point in array if pathing is randomized
        }
    }

    //find the closest patrol point when AI is not in range of player
    Vector3 GetClosestPatrolPoint()
    {
        Transform closest = null;

        foreach (Transform t in PatrolPoints)
        {
            if (closest == null)
            {
                closest = t;
            }
            
            else if (Vector3.Distance(transform.position, t.position) < Vector3.Distance(transform.position, closest.position))
            {
                closest = t;
            }
        }
        return closest.position;
    }
    #endregion

    #region FOV and perception
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
        visibleTargets.Clear(); //discards view of target from the FOV of AI
        Collider[] targetinViewRadius = Physics.OverlapSphere(transform.position, viewRadius, targetMask); //array of targets in the AI's FOV

        for (int i = 0; i < targetinViewRadius.Length; i++)
        {
            Transform player = targetinViewRadius[i].transform;
            Vector3 dirToTarget = (player.position - transform.position).normalized;

            if (Vector3.Angle(transform.forward, dirToTarget) < viewAngle / 2)
            {
                float dstToTarget = Vector3.Distance(transform.position, player.position);

                //If the player was not originally within the field of view
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
            if (i < vertexcount - 2)
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

    ViewCastInfo ViewCast(float globalAngle)
    {
        Vector3 direction = DirFromAngle(globalAngle, true);
        RaycastHit hit;

        if (Physics.Raycast(transform.position, direction, out hit, viewRadius, Unwalkable))
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

    //creates edges for Field of view
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
#endregion

    //draws wire framed objects in the level
    private void OnDrawGizmos()
    {
        //color for obstacles
        Gizmos.color = Color.red;
        Gizmos.DrawWireCube(boundbox.center, boundbox.size);

        //color and shape of patrol points
        Gizmos.color = Color.blue;
        Gizmos.DrawSphere(targetPos, 0.2f);

        //Gizmos.color = Color.yellow;
        //Gizmos.DrawWireSphere(transform.position, ChaseDistance);
    }
}
