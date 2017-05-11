using System.Collections;
using System.Collections.Generic;
using UnityEngine;
using UnityEngine.UI;

public class Menu : MonoBehaviour {

	private Text info;
	private InputField input;

	void Start() {
		Debug.Log("Menu");

		info = GameObject.Find ("IsConnectedTxt").GetComponent<Text> ();
		input = GameObject.Find ("InputField").GetComponent<InputField> ();

		GameObject.Find ("ToViewBtn").GetComponent<Button> ().onClick.AddListener(ToView);
		GameObject.Find ("ToLobbyBtn").GetComponent<Button> ().onClick.AddListener(ToLobby);

		info.text = "Not connected";
	}

	void Update(){


	}

	void ToView(){

		string ip = input.text;

		GAMEMANAGER.GM.Connect(ip);

		if (GAMEMANAGER.GM.GetSocketReady ()) {
			GAMEMANAGER.GM.SceneLoader ("View");
		} else {
			info.text = "Could not connect";
		}
	}

	void ToLobby(){

		string ip = input.text;

		GAMEMANAGER.GM.Connect(ip);

		if (GAMEMANAGER.GM.GetSocketReady ()) {
			GAMEMANAGER.GM.SceneLoader ("Lobby");
		} else {
			info.text = "Could not connect";
		}
	}
}
