using System.Collections;
using System.Linq;
using System.Collections.Generic;
using UnityEngine;

namespace NoamToledano.PathFinder
{
    public class PathFinder
    {
        public PathfinderGraph PathfinderGraph;

        private Dictionary<int, Vector3> _nodes;
        /// <summary>
        /// G = currentNode.g + distance between child and current
        /// H = distance from child to end
        /// F = child.g + child.h
        /// </summary>
        private Dictionary<int, GHF> _nodesGHF;
        /// <summary>
        /// first int = node, second int = parent
        /// </summary>
        private Dictionary<int, int> _nodesParents;

        private HashSet<int> _openSet;
        private HashSet<int> _closeSet;

        private int _iterationsLimit = 256;

        private struct GHF
        {
            public GHF(float g, float h, float f)
            {
                this.g = g;
                this.h = h;
                this.f = f;
            }
            public float g;
            public float h;
            public float f;
        }

        public PathFinder()
        {
            PathfinderGraph = new PathfinderGraph();

            _nodes                      = new Dictionary<int, Vector3>();
            _nodesGHF                   = new Dictionary<int, GHF>();
            _nodesParents               = new Dictionary<int, int>();
            _openSet                    = new HashSet<int>();
            _closeSet                   = new HashSet<int>();
        }

        /// <summary>
        /// returns empty list if no path has been found
        /// </summary>
        /// <param name="graph"></param>
        /// <param name="startNode"></param>
        /// <param name="targetNode"></param>
        /// <returns></returns>
        public List<Vector3> FindPath(Graph graph, int startNode, int targetNode)
        {
            InitializeSets(graph);
            _openSet.Add(startNode); // add start node
            _nodesGHF.Add(0, new GHF(0, 0, 0)); // set GHF to 0;

            int iteration = 0;
            while (_openSet.Count > 0 && iteration < _iterationsLimit)
            {
                // currentNode is the node with the samllest f value
                int currentNode = SetCurrentNode();

                if(currentNode == targetNode) // if this is the target
                {
                    return BacktracePath(currentNode, startNode);
                }

                List<Neighbor> childSet = graph.nodesNeighbors[currentNode];
                for (int i = 0; i < childSet.Count; i++)
                {
                    int child = childSet[i].index;
                    if (_closeSet.Contains(child))
                        continue;

                    // Create the ghf values
                    GHF ghf = new GHF();
                    ghf.g   = _nodesGHF[currentNode].g + childSet[i].distance;

                    if (_openSet.Contains(child) && _nodesGHF[child].g < ghf.g) // if child is in the open set && new g is bigger then stored g
                        continue;

                    ghf.h = Vector3.Distance(_nodes[child], _nodes[targetNode]); // this has been moved down for performance reasons
                    ghf.f = ghf.g + ghf.h;

                    _nodesGHF[child]     = ghf;
                    _nodesParents[child] = currentNode;

                    if (!_openSet.Contains(child))
                        _openSet.Add(child);
                }
            }

            return new List<Vector3>();
        }

        private int SetCurrentNode()
        {
            int currentNode = -1;
            float tempF = 100000;
            foreach (var nodeIndex in _openSet)
            {
                if (_nodesGHF[nodeIndex].f < tempF)
                {
                    tempF = _nodesGHF[nodeIndex].f;
                    currentNode = nodeIndex;
                }
            }

            _openSet.Remove(currentNode);
            _closeSet.Add(currentNode);

            return currentNode;
        }

        private void InitializeSets(Graph graph)
        {
            _openSet     .Clear();
            _closeSet    .Clear();
            _nodesGHF    .Clear();
            _nodesParents.Clear();

            _nodes = graph.nodes;
        }

        private List<Vector3> BacktracePath(int currentNode, int start)
        {
            List<Vector3> path = new List<Vector3>();
            path.Add(_nodes[currentNode]);

            while (currentNode != start)
            {
                currentNode = _nodesParents[currentNode];
                path.Add(_nodes[currentNode]);
            }

            path.Reverse();

            return path;
        }

        public void ClearData()
        {
            _nodes.Clear();
            PathfinderGraph.ClearMembers();
        }

        /* // A* (star) Pathfinding psudo code

        // Initialize both open and closed list
        let the openList equal empty list of nodes
        let the closedList equal empty list of nodes

     // Add the start node
        put the startNode on the openList (leave it's f at zero)

     // Loop until you find the end 
        while the openList is not empty 

        // Set the current node
            let the currentNode equal the node with the least f value
            remove the currentNode from the openList
            add the currentNode to the closedList

        // Found the goal
            if currentNode is the goal
                Congratz! You've found the end! Backtrack to get path

        // Generate children
            let the children of the currentNode equal the adjacent nodes
            (get adjacent nodes)

        //for each child in the children

            // Child is on the closedList
                if child is in the closedList
                    continue to beginning of for loop

            // Create the f, g, and h values
                child.g = currentNode.g + distance between child and current
                child.h = distance from child to end
                child.f = child.g + child.h

            // Child is already in openList
                if child.position is in the openList's nodes positions
                    if the child.g is higher than the openList node's g
                        continue to beginning of for loop

            // Add the child to the openList
            add the child to the openList
     */
    }
}