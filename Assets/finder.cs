using System.Collections;
using System.Collections.Generic;
using UnityEngine;

public class finder : MonoBehaviour {
	public Transform destinationPoint;

	// Use this for initialization

	void Start () {
		
	}
	
	// Update is called once per frame
	void Update () {
		transform.GetComponent<UnityEngine.AI.NavMeshAgent> ().destination = destinationPoint.position;

	}
}
