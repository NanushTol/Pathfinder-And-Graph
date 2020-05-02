using System.Collections;
using System.Collections.Generic;
using UnityEngine;

namespace NoamToledano.PathFinder
{
    public class PathfinderGraphTester : MonoBehaviour
    {
        #region Paramaters
        public float            SearchRadius;
        [Space]
        public GameObject[]     StartAndTargetNodes;
        [Space]
        public GraphParamaters  GraphParamaters;

        [Header("Debug Options")]
        public float            DisplayNodeRadius;
        public GameObject       QuadrentDummyPoint; 
        public bool             DrawPath;
        public bool             DrawNodes;
        public bool             DrawEdges;
        public bool             DrawQuadrent;
        public bool             DrawLines;
        public bool             DrawSearchRadius;

        public Dictionary<int, Vector3>         _nodes { get; private set; }
        public Dictionary<Vector2Int, float>    _edges;

        private PathFinder                      _pathFinder;
        private List<Vector3[]>                 _lines;

        private Graph                           _graph;
        private List<Vector3>                   _path;
        #endregion

        private void Start()
        {
            _pathFinder = new PathFinder();
            _lines      = new List<Vector3[]>();
        }

        private void Update()
        {
            CreateGraph();
            _path = GetPath();
        }

        public List<Vector3> GetPath()
        {
            return _pathFinder.FindPath(_graph, 0, 1);
        }

        public void CreateGraph()
        {
            Collider[] colliders = Physics.OverlapSphere(transform.position, SearchRadius, GraphParamaters.obstacleLayers);
            List<Transform> obstacles = new List<Transform>();

            for (int i = 0; i < colliders.Length; i++)
            {
                obstacles.Add(colliders[i].transform);
            }

            Vector3[] startAndTarget = new Vector3[] { StartAndTargetNodes[0].transform.position, StartAndTargetNodes[1].transform.position };

            if(DrawLines)
                _pathFinder.PathfinderGraph.SetDrawLines(true);

            Graph graph = _pathFinder.PathfinderGraph.CreateGraph(GraphParamaters, obstacles, startAndTarget, SearchRadius, transform.position);

            _nodes = graph.nodes;
            _edges = graph.edges;

            _graph = graph;

            if (DrawLines)
                _lines = _pathFinder.PathfinderGraph.GetLines();
        }

        private void OnDrawGizmos()
        {
            if (DrawSearchRadius)
            {
                Gizmos.color = Color.white;
                Gizmos.DrawWireSphere(transform.position, SearchRadius); 
            }

            if (_lines  != null && DrawLines)
            {
                Gizmos.color = Color.red;
                for (int i = 0; i < _lines.Count; i++)
                {
                    Gizmos.DrawLine(_lines[i][0], _lines[i][1]);
                }
            }

            if (_nodes  != null && DrawNodes)
            {
                Gizmos.color = Color.yellow;
                for (int i = 0; i < _nodes.Count; i++)
                {
                    if (i == 0)
                    {
                        Gizmos.color = Color.magenta;
                        Gizmos.DrawWireSphere(_nodes[i], DisplayNodeRadius);
                        Gizmos.color = Color.yellow;
                    }
                    else if(i == 1)
                    {
                        Gizmos.color = Color.green;
                        Gizmos.DrawWireSphere(_nodes[i], DisplayNodeRadius);
                        Gizmos.color = Color.yellow;
                    }
                    else
                        Gizmos.DrawWireSphere(_nodes[i], DisplayNodeRadius);
                }
            }

            if (_edges != null && DrawEdges)
            {
                Gizmos.color = Color.blue;
                foreach (var edge in _edges)
                {
                    Gizmos.DrawLine(_nodes[edge.Key.x], _nodes[edge.Key.y]);
                }
            }

            if (DrawQuadrent)
            {
                Gizmos.color = Color.white;
                Vector3 offset = new Vector3(GraphParamaters.gridSize / 2, 0, GraphParamaters.gridSize / 2);
                DrawQuadrentLines(QuadrentDummyPoint.transform.position, offset, GraphParamaters.edgeDistances.y);
            }

            if (DrawPath && _path != null && _path.Count > 0)
            {
                Gizmos.color = Color.black;
                for (int i = 0; i < _path.Count - 1; i++)
                {
                    Gizmos.DrawLine(_path[i], _path[i + 1]);
                }
            }
        }

        public void DrawQuadrentLines(Vector3 position, Vector3 offset, float quadSize)
        {
            if (_pathFinder != null)
            {
                int index = _pathFinder.PathfinderGraph.GetQuadrentIndexByPosition(position, offset);
                Debug.Log("Quadrent Index: " + index);
                if(_pathFinder.PathfinderGraph.GetQuadrents().ContainsKey(index))
                    Debug.Log("Quadrent Node Count: " + _pathFinder.PathfinderGraph.GetQuadrents()[index].Count);
                else
                    Debug.Log("Quadrent Node Count: 0");
            }

            position = position + offset;
            Vector3 lowerLeft = new Vector3(Mathf.FloorToInt(position.x / quadSize) * quadSize, 0, Mathf.FloorToInt(position.z / quadSize) * quadSize);
            Vector3 center = lowerLeft - offset + new Vector3(quadSize * 0.5f, 0, quadSize * 0.5f);
            Gizmos.DrawWireCube(center, new Vector3(quadSize, 0, quadSize));
            Gizmos.DrawWireCube(Vector3.zero, new Vector3(GraphParamaters.gridSize, 0, GraphParamaters.gridSize));
        }

        public void CheckNearby()
        {
            int index = _pathFinder.PathfinderGraph.GetQuadrentIndexByPosition(QuadrentDummyPoint.transform.position, new Vector3(GraphParamaters.gridSize / 2, 0, GraphParamaters.gridSize / 2));
            Debug.Log("nearby nodes: " + _pathFinder.PathfinderGraph.GetNearbyNodes(index).Count);
            Debug.Break();
        }
    }
}