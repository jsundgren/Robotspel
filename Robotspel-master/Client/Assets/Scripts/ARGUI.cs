﻿using UnityEngine;
using System.Collections;
using System;

public class ARGUI : MonoBehaviour
{
	private string fpsString;
	private GUIStyle fpsStyle;

	public void Start ()
	{
		fpsStyle = new GUIStyle ();
		fpsStyle.fontSize = 14;
		fpsStyle.normal.textColor = Color.black;
		fpsStyle.normal.textColor = Color.black;
		HUDFPS hs = FindObjectOfType (typeof(HUDFPS)) as HUDFPS;
		hs.OnFpsChange += fpsChange;//add the event listener here....
	}


	public void fpsChange (string s)
	{
		//event sent from HUDFPS
		fpsString = s;
	}

	private void drawFps ()
	{
		GUI.Label (new Rect (84f, 130f, 100f, 100f), fpsString, fpsStyle);
	}


	public void OnGUI ()
	{
		drawFps ();
	}

}