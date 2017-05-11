using System;
using System.Collections;
using System.Collections.Generic;
using System.Net.Sockets;
using UnityEngine;
using UnityEngine.SceneManagement;


public class GAMEMANAGER: MonoBehaviour
{

	public static GAMEMANAGER GM;

	private bool socketReady;
	private TcpClient client;
	private NetworkStream stream;

	List<string> Coords = new List<string>();

	void Awake ()
	{
		if (GM != null && GM != this) {
			DestroyImmediate (gameObject);
			return;
		}
		GM = this;
		DontDestroyOnLoad (gameObject);
	}

	void Start ()
	{
		InvokeRepeating("SocketConnected", 2.0f, 2.0f);
	}

	public void AddCoord(float x, float z){
		
		string data = x.ToString() + " " + z.ToString();
		Coords.Add (data);
	}

	public void SendCoords(){
		Coords.ForEach(SendString);
		Coords.Clear ();
	}

	public void Connect (string host)
	{
		
		if (socketReady)
			return;

		//Default IP/port
		int port = 6321;

		//Create socket
		try {
			client = new TcpClient (host, port);
			stream = client.GetStream ();
			socketReady = true;
		} catch (Exception e) {
			Debug.Log ("Socket error: " + e.Message);
		}
	}

	public void Disconnect ()
	{
		try {
			stream.Close ();
			client.Close ();
			socketReady = false;
		} catch (Exception e) {
			Debug.Log ("DC error: " + e.Message);
		}
	}

	public string Receive ()
	{
		if (stream.DataAvailable) {

			try {
				Byte[] data = new Byte[256];
				String responseData = String.Empty;
				Int32 bytes = stream.Read (data, 0, data.Length);
				responseData = System.Text.Encoding.ASCII.GetString (data, 0, bytes);
				return responseData;
			} catch (Exception e) {
				Debug.Log ("Receive error: " + e.Message);
				return "";
			}
		}

		return "";
	}

	public void SendString (string s)
	{
		Byte[] data = System.Text.Encoding.ASCII.GetBytes (s);

		try {
			stream.Write (data, 0, data.Length);
		} catch (Exception e) {
			Debug.Log ("# " + s);
			Debug.Log ("Socket error: " + e.Message);
		}
	}

	public void SocketConnected ()
	{
		if (socketReady) {
			bool part1 = client.Client.Poll (1000, SelectMode.SelectRead);
			bool part2 = (client.Client.Available == 0);
			if (part1 && part2) {
				socketReady = false;
			}
		}
	}

	public void SceneLoader (string s)
	{
		SceneManager.LoadScene (s);
	}

	public bool GetSocketReady ()
	{
		if (socketReady) {
			return true;
		} else {
			return false;
		}
	}
}
