using System.Collections;
using System.Collections.Generic;
using System.Diagnostics;
using UnityEngine;

public class Pathfinding : MonoBehaviour
{
    public Transform seeker; //the ai position
    public Transform target; //the player position
    MapGrid grid; //grid from mapgrid script

    void Awake()
    {
        grid = GetComponent<MapGrid>(); //get grid from MapGrid script
    }

    void FindPath(Vector3 startPos, Vector3 targetPos)
    {

        Node startNode = grid.NodeFromWorldPoint(startPos); //Ai position
        Node targetNode = grid.NodeFromWorldPoint(targetPos); //player position

        Heap<Node> openSet = new Heap<Node>(grid.MaxSize); //the set of nodes to be evaluated
        HashSet<Node> closedSet = new HashSet<Node>(); //the set of nodes that have already been evaluated
        openSet.Add(startNode); //add start node to open

        while (openSet.Count > 0) //while the current node = node in Open set with the lowest F_cost
        {
            Node currentNode = openSet.RemoveFirst(); //remove current from openSet
            closedSet.Add(currentNode); 

            if (currentNode == targetNode) 
            {
                RetracePath(startNode, targetNode); //path has been found
                return;
            }

            foreach (Node neighbour in grid.GetNeighbours(currentNode)) //for each neighbour in the current Node, 
            {
                if (!neighbour.walkable || closedSet.Contains(neighbour)) //if the neigbour is not walkable OR neigbour is in closedSet.
                {
                    continue; //skip to the next neigbour
                }

                int newCostToNeighbour = currentNode.gCost + GetDistance(currentNode, neighbour); //calculates new path
                if (newCostToNeighbour < neighbour.gCost || !openSet.Contains(neighbour)) //if new path to neighbour is shorter OR neighbour is not in OpenSet
                {
                    //set the f_cost of neigbour
                    neighbour.gCost = newCostToNeighbour;  
                    neighbour.hCost = GetDistance(neighbour, targetNode); 
                    neighbour.parent = currentNode; //set parent of neighbour to current

                    if (!openSet.Contains(neighbour)) //if neighbour is not in openSet
                        openSet.Add(neighbour); //add neigbour to open
                }
            }
        }
    }

    //retraces the path from the start node to the end node
    void RetracePath(Node startNode, Node endNode)
    {
        List<Node> path = new List<Node>(); //list of nodes for path
        Node currentNode = endNode; //end of the path

        while (currentNode != startNode) //while current node is not = to start node
        {
            path.Add(currentNode);
            currentNode = currentNode.parent;
        }
        path.Reverse();

        grid.path = path;
    }

    //get the distance between two nodes
    int GetDistance(Node nodeA, Node nodeB)
    {
        int dstX = Mathf.Abs(nodeA.gridX - nodeB.gridX); //distance on x axis
        int dstY = Mathf.Abs(nodeA.gridY - nodeB.gridY); //distance on y axis

        //if distance X is greater than distance Y
        if (dstX > dstY) 
            return 14 * dstY + 10 * (dstX - dstY);
        return 14 * dstX + 10 * (dstY - dstX);
    }

    // Start is called before the first frame update
    void Start()
    {
        
    }

    // Update is called once per frame
    void Update()
    {
        FindPath(seeker.position, target.position); //find a path between AI and player
    }
}
