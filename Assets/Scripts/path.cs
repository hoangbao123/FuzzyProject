using System.Collections;
using System.Collections.Generic;
using UnityEngine;

public class path : MonoBehaviour {
	public Color lineColor;
	private List<Transform> nodes = new List<Transform> ();
	// Use this for initialization
	void Start () {
		
	}
	
	// Update is called once per frame
	void Update () {
		
	}
	void OnDrawGizmos(){
		Transform[] path = GetComponentsInChildren<Transform> ();
		nodes = new List<Transform> ();
		for (int i = 0; i < path.Length; i++) {
			if (path [i].transform != transform) {
				nodes.Add (path [i]);

			} 

		}

		for (int i = 0; i < nodes.Count; i++) {
			Vector3 currentNodes, previousNode;
			currentNodes = nodes [i].position;
			previousNode = Vector3.zero;
			if (i > 0) {
				previousNode = nodes [i - 1].position;
			} else
				previousNode = nodes [nodes.Count - 1].position;
			Gizmos.DrawLine (currentNodes, previousNode);
			Gizmos.DrawWireSphere (currentNodes, 0.2f);
		}

	
	}
}
