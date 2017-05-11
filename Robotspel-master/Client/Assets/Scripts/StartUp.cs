using System.Collections;
using System.Collections.Generic;
using UnityEngine;
using UnityEngine.SceneManagement;

public class StartUp : MonoBehaviour {

	void Start () {
		Debug.Log("StartUp");
		SceneManager.LoadScene ("Menu");
	}
}
