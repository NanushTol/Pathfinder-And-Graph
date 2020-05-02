using System.Collections;
using System.Collections.Generic;
using UnityEngine;
using UnityEditor;
using System.IO;

public class CreateScriptableObject
{
	public T CreateAsset<T>(string path, string assetName, bool fullPath) where T : ScriptableObject
	{
		T asset = ScriptableObject.CreateInstance<T>();

		string assetPathAndName;
		if (fullPath)
			assetPathAndName = AssetDatabase.GenerateUniqueAssetPath(path);
		else
			assetPathAndName = AssetDatabase.GenerateUniqueAssetPath(path + assetName + ".asset");

		AssetDatabase.CreateAsset(asset, assetPathAndName);

		AssetDatabase.SaveAssets();
		AssetDatabase.Refresh();
		EditorUtility.FocusProjectWindow();
		Selection.activeObject = asset;

		return asset;
	}
}
