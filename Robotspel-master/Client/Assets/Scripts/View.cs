using System.Collections;
using System.Collections.Generic;
using UnityEngine;
using UnityEngine.UI;

public class View : MonoBehaviour {

	private Text info;
	private InputField input;



	void Start () {
		Debug.Log("View");

		info = GameObject.Find ("IsConnectedTxt").GetComponent<Text> ();
		input = GameObject.Find ("InputField").GetComponent<InputField> ();

		GameObject.Find ("DisconnectBtn").GetComponent<Button> ().onClick.AddListener(Disconnect);
		GameObject.Find ("SendStringBtn").GetComponent<Button> ().onClick.AddListener(SendStringInput);
		GameObject.Find ("SendCoordBtn").GetComponent<Button> ().onClick.AddListener(SendCoordInput);

		info.text = "Connected";
	}

	void Update(){
		if (GAMEMANAGER.GM.GetSocketReady ()) {
			
		} else {
			Disconnect ();
		}
	}

	void Disconnect(){

		GAMEMANAGER.GM.Disconnect ();
		GAMEMANAGER.GM.SceneLoader ("Menu");
	}

	void SendStringInput(){

		string msg = input.text;
		GAMEMANAGER.GM.SendString (msg);
	}

	void SendCoordInput(){

		GAMEMANAGER.GM.SendCoords ();
	}
}
