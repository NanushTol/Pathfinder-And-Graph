using System.Collections;
using System.Collections.Generic;
using UnityEngine;
using UnityEditor;

namespace NoamToledano.PathFinder
{
    [CustomEditor(typeof(PathfinderGraphTester))]
    public class PathFinderGraphTesterEditor : Editor
    {
        private int nodeCount = 0;

        private CreateScriptableObject _createScriptableObject = new CreateScriptableObject();

        public override void OnInspectorGUI()
        {
            PathfinderGraphTester pf = (PathfinderGraphTester)target;

            base.OnInspectorGUI();

            if (pf._nodes != null)
                nodeCount = pf._nodes.Count;
            else
                nodeCount = 0;

            EditorGUILayout.Space();
            EditorGUILayout.LabelField("Node Count: ", nodeCount.ToString());

            EditorGUILayout.Space();
            if (GUILayout.Button("Save Paramaters To File"))
            {
                SaveToScriptableObject(pf);
            }
        }

        private void SaveToScriptableObject(PathfinderGraphTester pf)
        {
            string folder = "/_Core Assets/Game Logic/Data/Pathfinder";
            string path = EditorUtility.SaveFilePanelInProject("Save Graph Paramaters", "Pathfinder Graph Param", "asset", "Please enter a file name", folder);
            
            PathfinderGraphSo asset = _createScriptableObject.CreateAsset<PathfinderGraphSo>(path, "", true);
            asset.GraphParamaters = pf.GraphParamaters;
        }
    }
}