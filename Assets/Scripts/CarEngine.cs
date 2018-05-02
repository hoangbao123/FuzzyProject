using System.Collections;
using System.Collections.Generic;
using UnityEngine;

public class CarEngine : MonoBehaviour {
	public Transform path;
	public float maxSteerAngle = 40f ; 	
	public float maxMotorTorque = 0f;
	public float currentSpeed;
	public float maxSpeed = 100f;
	public float turnSpeed = 100f;
	public WheelCollider wheelFL;
	public WheelCollider wheelFR;
	public WheelCollider wheelRL;
	public WheelCollider wheelRR;

	private List<Transform> nodes;
	private int currentNode = 0;
	private bool avoiding = false;
	private float targetSteerAngle = 0;
	private float balanceSteerAngle = 0;

	[Header("Sensor")]
	public float sensorLength = 5f;
	private float sideSensorLength = 20f;
	public Vector3 frontSensorPos = new Vector3(0,0.2f,1f); 
	public float sizeSensorPos = 0.9f;
	public float frontSensorAngle = 30;
	public Vector3 rightSensorPos;
	public Vector3 leftSensorPos;
	private float distanceFromLeft = 0;
	private float distanceFromRight = 0;
	private float distanceFromObs = 0;




	// Use this for initialization
	void Start () {
		Transform[] pathTransforms = path.GetComponentsInChildren<Transform> ();
		nodes = new List<Transform> ();
		for (int i = 0; i < pathTransforms.Length; i++) {
			if(pathTransforms[i] !=path.transform)
				nodes.Add (pathTransforms [i]);
			
		}
	}
	
	// Update is called once per frame
	void Update () {
		
	}
	void FixedUpdate(){
		Sensors ();
		ApplySteering ();
		Drive ();
		CheckWayPointDistance ();
		FuzzyInferrence ();
		LerpToSteerAngle ();
	}
	void OnDrawGizmos(){
		Gizmos.DrawWireSphere (rightSensorPos, 0.1f);
		Gizmos.DrawWireSphere (leftSensorPos, 0.1f);
	}
	void ApplySteering(){
		
		if (avoiding)
			return;
		Vector3 relativeVector = transform.InverseTransformPoint (nodes [currentNode].position);

		float newSteer =  (relativeVector.x/relativeVector.magnitude)*maxSteerAngle;
		targetSteerAngle = newSteer;
	//	wheelFL.steerAngle = newSteer;
		//wheelFR.steerAngle = newSteer;
	}
	void Drive (){
		currentSpeed = 2 * Mathf.PI * wheelFL.radius * wheelFL.rpm * 60 / 1000;
		if (currentSpeed < maxSpeed) {
			wheelFL.motorTorque = maxMotorTorque;
			wheelFR.motorTorque = maxMotorTorque;
		} else {
			wheelFL.motorTorque = 0;
			wheelFR.motorTorque = 0;
		}
	}
	void CheckWayPointDistance(){
		print ("current node" + currentNode);
		print ("distance : " + Vector3.Distance (transform.position, nodes [currentNode].position));
		if (Vector3.Distance (transform.position, nodes [currentNode].position) < 3f) {
			if (currentNode == nodes.Count - 1)
				currentNode = 0;
			else
				currentNode++;
			
		}
	}
	void Sensors(){
		RaycastHit hit;

		float avoidMultiplier = 0;

		avoiding = false;


		//right sensor
		rightSensorPos = transform.position + transform.right*sizeSensorPos;
		Debug.DrawRay (rightSensorPos, transform.right);
		print("right sensor "+rightSensorPos);
		if (Physics.Raycast (rightSensorPos, transform.right, out hit, sideSensorLength)) {
			if (hit.collider.CompareTag ("fence")) {
				Debug.DrawLine (rightSensorPos, hit.point);
				distanceFromRight = hit.distance;
				print ("right:" + distanceFromRight);
			}
		}
		//left sensor
		leftSensorPos = transform.position + transform.right*sizeSensorPos*-1;
		Debug.DrawRay (leftSensorPos, transform.right * -1);
		print(" sensor left "+leftSensorPos);
		if (Physics.Raycast (leftSensorPos, transform.right*-1, out hit, sideSensorLength)) {
			if (hit.collider.CompareTag ("fence")) {
				Debug.DrawLine (leftSensorPos, hit.point);
				distanceFromLeft = hit.distance;
				print ("left:" + distanceFromLeft);
			}
		}

		Vector3 sensorStartingPos = transform.position ;
		sensorStartingPos += transform.forward * frontSensorPos.z;
		sensorStartingPos += transform.up * frontSensorPos.y;
		//front right sensor

		sensorStartingPos += transform.right*sizeSensorPos; 
		//print("right center:"+sensorStartingPos);
		if (Physics.Raycast (sensorStartingPos, transform.forward, out hit, sensorLength)) {
			if(hit.collider.CompareTag("obstacle")){
				Debug.DrawLine (sensorStartingPos, hit.point);
				avoiding = true;
				avoidMultiplier = -1f;

			}

		}
		//front right angle sensor
		else if (Physics.Raycast (sensorStartingPos, Quaternion.AngleAxis(frontSensorAngle,transform.up)*transform.forward, out hit, sensorLength)) {
			if(hit.collider.CompareTag("obstacle")){
				Debug.DrawLine (sensorStartingPos, hit.point);

				avoiding = true;
				avoidMultiplier = -0.5f;
			}
		}
		//

		//front left sensor

		sensorStartingPos -= 2*transform.right*sizeSensorPos;
	//	print("left center:"+sensorStartingPos);
		if (Physics.Raycast (sensorStartingPos, transform.forward, out hit, sensorLength)) {

			if(hit.collider.CompareTag("obstacle")){
				Debug.DrawLine (sensorStartingPos, hit.point);
				avoiding = true;
				avoidMultiplier = 1f;
			}
		}
		//front left angle sensor
		if (Physics.Raycast (sensorStartingPos, Quaternion.AngleAxis (-frontSensorAngle, transform.up) * transform.forward, out hit, sensorLength)) {
			if (hit.collider.CompareTag ("obstacle")) {
				Debug.DrawLine (sensorStartingPos, hit.point);
				avoiding = true;
				avoidMultiplier = 0.5f;
			}

		}
		//front center sensor

		if (avoidMultiplier == 0) {
			if (Physics.Raycast (sensorStartingPos, transform.forward, out hit, sensorLength)) {
				if (hit.collider.CompareTag ("obstacle")) {
					Debug.DrawLine (sensorStartingPos, hit.point);
					avoiding = true;
					if (hit.normal.x < 0) {
						avoidMultiplier = -1f;

					} else
						avoidMultiplier = 1f;
				}
			}
		}

		if (avoiding) {
			targetSteerAngle = maxSteerAngle * avoidMultiplier; 

		}
	}
	void LerpToSteerAngle(){
		
		wheelFL.steerAngle = Mathf.Lerp (wheelFL.steerAngle, targetSteerAngle, Time.deltaTime * turnSpeed);
		print ("dt time:" + Time.deltaTime);
		print ("dt time*turn:" + Time.deltaTime * turnSpeed);
		wheelFL.steerAngle = Mathf.Lerp (wheelFL.steerAngle, targetSteerAngle, Time.deltaTime * turnSpeed);
		print ("Whee fl:" + wheelFL.steerAngle);

		//wheelFL.steerAngle = targetSteerAngle;
		//wheelFR.steerAngle = targetSteerAngle;
		print ("steer:" + targetSteerAngle);
	}
	void FuzzyInferrence(){
		float difference = distanceFromLeft - distanceFromRight;
		float leftVsRight = difference / (distanceFromLeft + distanceFromRight);
		print ("ti le :" + leftVsRight);

	
		if (leftVsRight < -0.8f)
			balanceSteerAngle = 0.2f;
		else if (leftVsRight < -0.2f)
			balanceSteerAngle = 0.1f;
		else if (leftVsRight < 0.3f)
			balanceSteerAngle = 0;
		else if (leftVsRight < 0.6f)
			balanceSteerAngle = -0.1f;
		else
			balanceSteerAngle = -0.2f;

	}

}

