#pragma once

#include <string>
#include "SpheroRAWItf.h"
#include <windows.h>
#include <iostream>
#include <sstream>

#using <System.dll>

public ref class SpheroLogic
{
private:
	ISpheroDevice* device;
	std::vector<std::pair<float, float>>* targetPositions;
	float X, Y;
	bool moving;

	void PrintDeviceStatus(std::string action, ISpheroDevice* device);
	float distToPoint(float X, float Y, float Xtarget, float Ytarget);
	void calculatePath(std::pair<float,float> target);
	
	int getAngle(std::pair<float, float> target);

public:
	SpheroLogic(const char* name);
	~SpheroLogic();
	int offAngle;

	void moveSphero();
	void keyMove();
	void setTarget(std::string targetString);
	bool spheroConnected();
	void printDeviceStatus(std::string action);
	void updateSpheroPos(float Xpos, float Ypos);
	void testMove();
	void setOrientation();
};

