using System.Collections;
using System.Linq;
using System.Collections.Generic;
using UnityEngine;
using TheTraveler;

namespace NoamToledano.PathFinder
{
    public class PathfinderGraph
    {
        public PathfinderGraph()
        {
            _obstaclesPoints        = new Dictionary<int, Vector3>();
            _obstacles              = new Dictionary<int, Transform>();
            _obstaclesPointsLookup  = new Dictionary<Vector3, int>();
            _lines                  = new List<Vector3[]>();
            _nodes                  = new Dictionary<int, Vector3>();
            _nodeNeighbors          = new Dictionary<int, List<Neighbor>>();
            _quadrentNodes          = new Dictionary<int, List<int>>();
            _edges                  = new Dictionary<Vector2Int, float>();
            _inverseEdges           = new Dictionary<Vector2Int, float>();
        }

        #region Paramaters
        // node creation
        private LayerMask    _obstacleLayers;
        private float        _searchRaduis;
        private Vector3      _searchCenter;
        private Vector2      _obstacleDistances;
        private int          _pointsInCircle;
        private int          _circlesCount;
        private float        _circlesOffset;
        private bool         _drawLines;

        // quadrent
        private float        _quadrentSize;
        private float        _gridSize;

        // edges
        private Vector2      _edgeDistances;
        private float        _agentDiameter;
        private int          _edgeCount;

        // start & Target
        private Vector3[]    _startAndTargetNodes;

        private float       _castAngleStep;
        private int         _quadrentCount;
        private Vector3     _offset;
        #endregion

        #region Data Sets
        private Dictionary<int, Vector3>        _obstaclesPoints;
        private Dictionary<Vector3, int>        _obstaclesPointsLookup;
        private Dictionary<int, Transform>      _obstacles;
        private List<Vector3[]>                 _lines;
        private Dictionary<int, Vector3>        _nodes;
        private Dictionary<int, List<Neighbor>> _nodeNeighbors;
        private Dictionary<int, List<int>>      _quadrentNodes;             // key = quadrents, value = nodes indecies list
        private Dictionary<Vector2Int, float>   _edges;                     // int = node index
        private Dictionary<Vector2Int, float>   _inverseEdges;                     // int = node index
        #endregion

        public Graph CreateGraph(GraphParamaters graphParamaters, List<Transform> obstacles, Vector3[] startAndTarget, float searchRadius, Vector3 searchCenter)
        {
            SetupGraph(graphParamaters, obstacles, startAndTarget, searchRadius, searchCenter);
            CreateNodes();
            CreateEdges();

            return GetGraph();
        }

        #region Setup Functions
        private void SetupGraph(GraphParamaters graphParamaters, List<Transform> obstacles, Vector3[] startAndTarget, float searchRadius, Vector3 searchCenter)
        {
                ClearMembers();

                SetupObstacles(obstacles);

                _obstacleLayers         = graphParamaters.obstacleLayers;
                _searchRaduis           = searchRadius;
                _searchCenter           = searchCenter;
                _obstacleDistances      = graphParamaters.obstacleDistances;
                _pointsInCircle         = graphParamaters.pointsInCircle;
                _circlesCount           = graphParamaters.circlesCount;
                _circlesOffset          = graphParamaters.circlesOffset;
                _gridSize               = graphParamaters.gridSize;

                _edgeDistances          = graphParamaters.edgeDistances;
                _agentDiameter          = graphParamaters.agentDiameter;
                _edgeCount              = graphParamaters.edgeCount;

                _startAndTargetNodes    = startAndTarget;

                SetupQuadrents();

                _castAngleStep = 360f / _pointsInCircle;
        }

        public  void SetupQuadrents()
        {
                _quadrentSize  = _edgeDistances.y;
                _quadrentCount = Mathf.FloorToInt(_gridSize / _quadrentSize);
                _offset        = new Vector3(_gridSize / 2, 0, _gridSize / 2);
        }

        public  void SetupObstacles(List<Transform> obstacles)
        {
            for (int i = 0; i < obstacles.Count; i++)
                {
                _obstaclesPoints      .Add(i, obstacles[i].position);
                _obstaclesPointsLookup.Add(obstacles[i].position, i);
                _obstacles            .Add(i, obstacles[i]);
                }
        }

        public  void ClearMembers()
        {
            _obstacles            .Clear();
            _obstaclesPoints      .Clear();
            _obstaclesPointsLookup.Clear();
            _lines                .Clear();
            _nodes                .Clear();
            _quadrentNodes        .Clear();
            _edges                .Clear();
            _inverseEdges         .Clear();
            _nodeNeighbors        .Clear();
        }
        #endregion

        #region Graph Creation Functions
        public void CreateNodes()
        {
            int nodeIndex = 2;

            AddNodeToGraph(0, _startAndTargetNodes[0]); // Start node
            AddNodeToGraph(1, _startAndTargetNodes[1]); // Target node

            for (int o = 0; o < _obstacles.Count; o++) // create obstacles node network
            {
                Vector3 origin = _obstaclesPoints[o];
                float range = _obstacleDistances.y - _obstacleDistances.x;
                float distStep = 1f / _circlesCount;

                for (int c = 0; c < _circlesCount; c++) // create circle
                {
                    float dist = _obstacleDistances.x + range * distStep * c + _obstacles[o].lossyScale.x * 0.5f;
                    Vector3[] circlePoints = CreatePointsCircle(dist, _pointsInCircle, _castAngleStep, c * _castAngleStep * _circlesOffset);

                    for (int p = 0; p < circlePoints.Length; p++)
                    {
                        Vector3 nodePos = origin + circlePoints[p];

                        if (_drawLines)
                            _lines.Add(new Vector3[2] { origin, nodePos });

                        if (CheckNodeLocation(nodePos))
                        {
                            AddNodeToGraph(nodeIndex, nodePos);
                            nodeIndex++;
                        }
                    }
                }
            }

            CreateUniformCircularGrid(nodeIndex);
        }

        private void CreateUniformCircularGrid(int nodeIndex)
        {
            float stepSize   = _edgeDistances.y * 0.75f;
            int circlesCount = Mathf.FloorToInt(_searchRaduis / stepSize);

            if (CheckNodeLocation(_searchCenter))
            {
                AddNodeToGraph(nodeIndex, _searchCenter);
                nodeIndex++;
            }
            for (int i = 1; i <= circlesCount; i++)
            {
                float stepAngle  = FindStepAngle(stepSize * i, stepSize);
                Vector3[] points = CreatePointsCircle(stepSize * i, Mathf.CeilToInt(360f / stepAngle), stepAngle, 0f);

                for (int p = 0; p < points.Length; p++)
                {
                    if (CheckNodeLocation(_searchCenter + points[p]))
                    {
                        AddNodeToGraph(nodeIndex, _searchCenter + points[p]);
                        nodeIndex++;
                    }
                }
            }
        }

        private bool CheckNodeLocation(Vector3 nodePos)
        {
            if (Vector3.Distance(nodePos, _searchCenter) > _searchRaduis)
                return false;

            if (!Physics.CheckSphere(nodePos, _agentDiameter * 0.5f, _obstacleLayers))
            {
                   List< int > nearbyNodes = GetNearbyNodes(nodePos);
                if (CheckRadius(nodePos, _edgeDistances.x, nearbyNodes, out int hitIndex) && hitIndex != 1 && hitIndex != 0) // dont check for start & end nodes
                {
                    // mid point:  A+(B-A)/2
                    Vector3 midPoint = _nodes[hitIndex] + (nodePos - _nodes[hitIndex]) * 0.5f;
                    _nodes[hitIndex] = midPoint;

                    return false; // re-set the close node position, dont create new node
                }
                else
                {
                    return true; // create new node
                }
            }
            return false; // no vecant place was found
        }

        private void AddNodeToGraph(int nodeIndex, Vector3 nodePos)
        {
            _nodes.Add(nodeIndex, nodePos);
            int quadIndex = GetQuadrentIndexByPosition(nodePos, _offset);
            if (_quadrentNodes.ContainsKey(quadIndex))
                _quadrentNodes[quadIndex].Add(nodeIndex);
            else
                _quadrentNodes.Add(quadIndex, new List<int>() { nodeIndex });
        }

        private void CreateEdges()
        {
            for (int i = 0; i < _nodes.Count; i++)    
            {
                List<int> nearbyNodes = GetNearbyNodes(_nodes[i]);

                FindNeighbors(i, _edgeCount, nearbyNodes);

                List<Neighbor> neighborList = _nodeNeighbors[i];
                foreach (var neighbor in neighborList)
                {
                    Vector2Int edge        = new Vector2Int(i, neighbor.index);
                    Vector2Int inverseEdge = new Vector2Int(neighbor.index, i);
                    if (!_edges.ContainsKey(edge) && !_inverseEdges.ContainsKey(edge))
                    {
                        _edges.Add(edge, neighbor.distance);
                        _inverseEdges.Add(inverseEdge, neighbor.distance);
                    }
                } 
            }
        }

        private void FindNeighbors(int nodeIndex, int count, List<int> nearbyNodes)
        {
            if (!_nodeNeighbors.ContainsKey(nodeIndex)) 
            {
                _nodeNeighbors.Add(nodeIndex, new List<Neighbor>());
                for (int i = 0; i < count; i++)
                {
                    _nodeNeighbors[nodeIndex].Add(new Neighbor() { index = -1, distance = 10000f });
                }
            }

            List<Neighbor> closestneighbors = _nodeNeighbors[nodeIndex];

            for (int i = 0; i < nearbyNodes.Count; i++)
            {
                int nearbyIndex = nearbyNodes[i];
                Vector3 nearbyPos = _nodes[nearbyIndex];

                float distance = Vector3.Distance(nearbyPos, _nodes[nodeIndex]);
                if(nodeIndex >= 2) //  Exclude start & target from distance limitation & neighbors count limit
                {
                    // check near node popularity
                    if (_nodeNeighbors.ContainsKey(nearbyIndex) && _nodeNeighbors[nearbyIndex].Count >= count)
                        continue;

                    // verify distance
                    if (distance > _edgeDistances.y || distance < _edgeDistances.x)
                        continue;
                }

                // verify direct line of sight
                Vector3 direction = _nodes[nearbyIndex] - _nodes[nodeIndex];
                if (Physics.SphereCast(_nodes[nodeIndex], _agentDiameter * 0.5f, direction, out RaycastHit hit, distance, _obstacleLayers))
                    continue;

                for (int j = 0; j < count; j++)
                {
                    if (distance < closestneighbors[j].distance)
                    {
                        closestneighbors.Insert(j, new Neighbor() { index = nearbyIndex, distance = distance});
                        
                        if (closestneighbors.Count > count)
                            closestneighbors.RemoveRange(count - 1, closestneighbors.Count - count - 1);

                        break;
                    }
                }
            }

            for (int i = closestneighbors.Count - 1; i > -1; i--)
            {
                if (closestneighbors[i].index == -1)
                    closestneighbors.RemoveAt(i);
            }

            _nodeNeighbors[nodeIndex] = closestneighbors; // assign closest to my neighbors dic
        }

        private bool CheckRadius(Vector3 position, float raduis, List<int> nearbyNodes, out int hitIndex)
        {
            for (int i = 0; i < nearbyNodes.Count; i++)
            {
                if (Vector3.Distance(position, _nodes[nearbyNodes[i]]) < raduis)
                {
                    hitIndex = nearbyNodes[i];
                    return true;
                }
            }

            hitIndex = -1;
            return false;
        }

        private Vector3[] CreatePointsCircle(float radius, int pointsCount, float stepAngle, float startAngle)
        {
            Vector3[] points = new Vector3[pointsCount];
            float theta = startAngle;

            for (int i = 0; i < pointsCount; i++)
            {
                float xPos = radius * Mathf.Cos(theta * Mathf.Deg2Rad);
                float yPos = radius * Mathf.Sin(theta * Mathf.Deg2Rad);

                points[i] = new Vector3(xPos, 0, yPos);

                theta += stepAngle;
            }

            return points;
        }

        /// <summary>
        /// Finds the angle of a triangle based on its base size & radius
        /// </summary>
        /// <param name="radius"></param>
        /// <param name="stepSize"></param>
        /// <returns></returns>
        public static float FindStepAngle(float radius, float stepSize)
        {
            float a = stepSize;
            float b = radius;
            float c = radius;

            float step1 = (b * b) + (c * c) - (a * a);
            float step2 = 2f * b * c;
            float angle = Mathf.Acos(step1 / step2) * Mathf.Rad2Deg;

            return angle;
        }
        #endregion

        #region Private Get Functions
        private Graph       GetGraph()
        {
            Graph graph = new Graph();
            graph.nodes = _nodes;
            graph.nodesNeighbors = _nodeNeighbors;
            graph.edges = _edges;

            return graph;
        }

        public List<int>    GetNearbyNodes(int nodeIndex) // used for debug
        {
            Dictionary<int, List<int>> closeQuadrents = GetCloseQuadrents(_nodes[nodeIndex], _edgeDistances.y);
            List<int> nodesIndecies = new List<int>();

            foreach (var indexList in closeQuadrents)
            {
                for (int n = 0; n < indexList.Value.Count; n++) // for each node index in list
                {
                    if (indexList.Value[n] != nodeIndex)
                        nodesIndecies.Add(indexList.Value[n]);
                }
            }

            return nodesIndecies;
        }

        private List<int>   GetNearbyNodes(Vector3 nodePos)
        {
            float factor = 1f;
            // exclude start & target from distance limitation by searching farther quadrents
            if (nodePos == _startAndTargetNodes[0] || nodePos == _startAndTargetNodes[1]) 
                factor = 1f;

            Dictionary<int, List<int>> closeQuadrents = GetCloseQuadrents(nodePos, _edgeDistances.y * factor);
            List<int> nodesIndecies = new List<int>();

            foreach (var indexList in closeQuadrents) // for each quadrent
            {
                for (int n = 0; n < indexList.Value.Count; n++) // for each node index in list
                {
                    nodesIndecies.Add(indexList.Value[n]);
                }
            }
            return nodesIndecies;
        }

        private Dictionary<int, List<int>> GetCloseQuadrents(Vector3 nodePos, float radius)
        {
            Dictionary<int, List<int>> quadrents = new Dictionary<int, List<int>>();
            int index;

            index = GetQuadrentIndexByPosition(nodePos, _offset);
            if (_quadrentNodes.ContainsKey(index))
                quadrents.Add(index, _quadrentNodes[index]);


            Vector3 upPos = nodePos + new Vector3(0, 0, radius);
            index = GetQuadrentIndexByPosition(upPos, _offset);

            if (_quadrentNodes.ContainsKey(index) && !quadrents.ContainsKey(index))
                quadrents.Add(index, _quadrentNodes[index]);
            //Debug.Log("up quadrent: " + index);


            Vector3 upRightPos = nodePos + new Vector3(radius, 0, radius);
            index = GetQuadrentIndexByPosition(upRightPos, _offset);

            if (_quadrentNodes.ContainsKey(index) && !quadrents.ContainsKey(index))
                quadrents.Add(index, _quadrentNodes[index]);
            //Debug.Log("up right quadrent: " + index);


            Vector3 rightPos = nodePos + new Vector3(radius, 0, 0);
            index = GetQuadrentIndexByPosition(rightPos, _offset);

            if (_quadrentNodes.ContainsKey(index) && !quadrents.ContainsKey(index))
                quadrents.Add(index, _quadrentNodes[index]);
            //Debug.Log("right quadrent: " + index);


            Vector3 downRightPos = nodePos + new Vector3(radius, 0, -radius);
            index = GetQuadrentIndexByPosition(downRightPos, _offset);

            if (_quadrentNodes.ContainsKey(index) && !quadrents.ContainsKey(index))
                quadrents.Add(index, _quadrentNodes[index]);
            //Debug.Log("down right quadrent: " + index);


            Vector3 downPos = nodePos + new Vector3(0, 0, -radius);
            index = GetQuadrentIndexByPosition(downPos, _offset);

            if (_quadrentNodes.ContainsKey(index) && !quadrents.ContainsKey(index))
                quadrents.Add(index, _quadrentNodes[index]);
            //Debug.Log("down quadrent: " + index);


            Vector3 downLeftPos = nodePos + new Vector3(-radius, 0, -radius);
            index = GetQuadrentIndexByPosition(downLeftPos, _offset);

            if (_quadrentNodes.ContainsKey(index) && !quadrents.ContainsKey(index))
                quadrents.Add(index, _quadrentNodes[index]);
            //Debug.Log("down left quadrent: " + index);


            Vector3 leftPos = nodePos + new Vector3(-radius, 0, 0);
            index = GetQuadrentIndexByPosition(leftPos, _offset);

            if (_quadrentNodes.ContainsKey(index) && !quadrents.ContainsKey(index))
                quadrents.Add(index, _quadrentNodes[index]);
            //Debug.Log("left quadrent: " + index);


            Vector3 upLeftPos = nodePos + new Vector3(-radius, 0, radius);
            index = GetQuadrentIndexByPosition(upLeftPos, _offset);

            if (_quadrentNodes.ContainsKey(index) && !quadrents.ContainsKey(index))
                quadrents.Add(index, _quadrentNodes[index]);
            //Debug.Log("Up left quadrent: " + index);

            return quadrents;
        }
        #endregion

        #region Public Get Functions
        public int GetQuadrentIndexByPosition(Vector3 position, Vector3 offset)
        {
            position += offset;
            return Mathf.FloorToInt(position.x / _quadrentSize) + (_quadrentCount * Mathf.FloorToInt(position.z / _quadrentSize));
        }

        public List<Vector3[]> GetLines()
        {
            List<Vector3[]> lines = new List<Vector3[]>();
            for (int i = 0; i < _lines.Count; i++)
            {
                Vector3[] line = new Vector3[2]
                {
                    _lines[i][0],
                    _lines[i][1]
                };
                lines.Add(line);
            }
            return lines;
        }

        public Dictionary<int, Vector3> GetNodes()
        {
            return _nodes;
        }

        public Dictionary<int, List<int>> GetQuadrents()
        {
            return _quadrentNodes;
        }

        public Dictionary<Vector2Int, float> GetEdges()
        {
            return _edges;
        }
        #endregion

        public void SetDrawLines(bool drawLines)
        {
            _drawLines = drawLines;
        }
    }

    #region Structs
    public struct Neighbor
    {
        public Neighbor(int index, float dist)
        {
            this.index = index;
            distance = dist;
        }
        public int index;
        public float distance;
    };

    [System.Serializable]
    public struct GraphParamaters
    {
        [Header("Layers")]
        public LayerMask obstacleLayers;

        [Header("Node Creation")]
        public Vector2 obstacleDistances;
        [Min(1)]
        public int pointsInCircle;
        [Min(1)]
        public int circlesCount;
        [Range(0f, 1f)]
        public float circlesOffset;

        [Header("Agent")]
        public float agentDiameter;

        [Header("Edge Creation")]
        public Vector2 edgeDistances;
        public int edgeCount;

        [Header("Quadrent System")]
        public float gridSize;
    };

    public struct Graph
    {
        public Dictionary<int, Vector3> nodes;
        public Dictionary<int, List<Neighbor>> nodesNeighbors;

        /// <summary> Vector2Int = nodes indecies, float = edge lenght </summary>
        public Dictionary<Vector2Int, float> edges;
    }
    #endregion
}