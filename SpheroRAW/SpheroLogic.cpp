#include "SpheroLogic.h"



SpheroLogic::SpheroLogic(const char* name)
{
	device = SpheroRAW_Create(name);
	while(!spheroConnected())
		device->connect();
	PrintDeviceStatus("Connecting: ", device);
	device->setAutoReconnect();
	device->abortMacro();
	targetPositions = new std::vector<std::pair<float, float>>;
	moving = false;
}


SpheroLogic::~SpheroLogic()
{
	delete targetPositions;
	SpheroRAW_Destroy(device);
}



void SpheroLogic::moveSphero()
{
	//std::vector<SpheroMessage> messages = device->receive();
	if (!moving)
	{
		float dist;
		moving = true;
		for (auto target : *targetPositions) {
			do {
				calculatePath(target);
				Sleep(500);
				dist = distToPoint(X, Y, target.first, target.second);
			} while (dist > 10.0f);
		}
		moving = false;
	}
	else
	{
		std::cout << "Already issuing move command to sphero" << std::endl;
		return;
	}
}

void SpheroLogic::keyMove()
{
	if (GetAsyncKeyState('W'))
	{
		//printDeviceStatus("Test");
		device->eraseOrbBasicStorage(0x0);
		device->abortMacro();
		device->roll(100, 0, 2);
		
	}
	else if (GetAsyncKeyState('S'))
	{
		device->eraseOrbBasicStorage(0x0);
		device->abortMacro();
		//device->eraseOrbBasicStorage(0x0);
		device->roll(100, 180, 1);
	}
	else if (GetAsyncKeyState('A'))
	{
		device->eraseOrbBasicStorage(0x0);
		device->abortMacro();
		//device->eraseOrbBasicStorage(0x0);
		device->roll(100, 270, 1);
	}
	else if (GetAsyncKeyState('D'))
	{
		device->eraseOrbBasicStorage(0x0);
		device->abortMacro();
		//device->eraseOrbBasicStorage(0x0);
		device->roll(100, 90, 1);
	}
	else if (GetAsyncKeyState('E'))
	{
		device->eraseOrbBasicStorage(0x0);
		CommandParameters data;
		data.clear();
		data.push_back(0x0);
		data.push_back(0x0);
		data.push_back(0x0);
		data.push_back(0x0);
		data.push_back(0x0);
		data.push_back(0x0);
		device->configureCollisionDetection(data);
		device->setStabilisation(true);
	}
	else if (GetAsyncKeyState('R'))
	{
		device->abortMacro();
		device->setRGBLedOutput(255, 0, 0, true);
	}
	else if (GetAsyncKeyState('G'))
	{
		device->abortMacro();
		device->setRGBLedOutput(0, 255, 0, true);
	}
	else if (GetAsyncKeyState('B'))
	{
		device->abortMacro();
		device->setRGBLedOutput(0, 0, 255, true);
	}
	else if (GetAsyncKeyState('T'))
	{
		
		device->abortMacro();
		device->setRGBLedOutput(255, 255, 255, true);
		device->setHeading((ushort)270);
	}
	else
	{
		device->eraseOrbBasicStorage(0x0);
		device->abortMacro();
		//device->eraseOrbBasicStorage(0x0);
		//device->roll(0, 0, 0);
	}
}

void SpheroLogic::setTarget(std::string targetString)
{
	if (!moving)
	{
		float x, y;
		std::stringstream stream = std::stringstream(targetString);
		targetPositions->clear();
		while (stream >> x && stream >> y)
		{
			std::cout << -y << "     " << x << std::endl;
			targetPositions->push_back(std::make_pair(-y * 100.0f, x*100.0f));
		}
	}
}

bool SpheroLogic::spheroConnected()
{
	if(device->state() == SpheroState_Connected)
		return true;

	return false;
}

void SpheroLogic::printDeviceStatus(std::string action)
{
	PrintDeviceStatus(action, device);
}

void SpheroLogic::testMove()
{
	int angle = getAngle(targetPositions->at(0));
	std::cout << "Sphero Position: " << X << " " << Y << std::endl;
	std::cout << "Target Position: " << targetPositions->at(0).first << " " << targetPositions->at(0).second << std::endl;
	device->abortMacro();
	device->roll(60, angle, 1);
}

void SpheroLogic::setOrientation()
{
	device->abortMacro();
	device->roll(50, 0, 1);
	device->setRGBLedOutput(255, 0, 0, 1);

	int offsetAngle;
	std::cin >> offsetAngle;
	offAngle = offsetAngle;
	//Sleep(1000);

	device->abortMacro();
	device->roll(50, 360 - offsetAngle, 1);
}


void SpheroLogic::PrintDeviceStatus(std::string action, ISpheroDevice * device)
{
	std::cout << "Action: " << action << " Result: ";

	if (device == nullptr) {
		std::cout << "Error - Sphero handle is invalid" << std::endl;
		return;
	}

	switch (device->state()) {
	case SpheroState_None: { std::cout << "SpheroRAW not initialized" << std::endl; break; }
	case SpheroState_Error_BluetoothError: { std::cout << "Error - Couldn't initialize Bluetooth stack" << std::endl; break; }
	case SpheroState_Error_BluetoothUnavailable: { std::cout << "Error - No valid Bluetooth adapter found" << std::endl; break; }
	case SpheroState_Error_NotPaired: { std::cout << "Error - Specified Sphero not Paired" << std::endl; break; }
	case SpheroState_Error_ConnectionFailed: { std::cout << "Error - Connecting failed" << std::endl; break; }
	case SpheroState_Disconnected: { std::cout << "Sphero disconnected" << std::endl; break; }
	case SpheroState_Connected: { std::cout << "Sphero connected" << std::endl; break; }
	}

	std::cout << std::endl;
}


float SpheroLogic::distToPoint(float X1, float Y1, float Xtarget, float Ytarget)
{
	return sqrt(pow(X1-Xtarget, 2.0f) + pow(Y1-Ytarget,2.0f));
}

//
void SpheroLogic::calculatePath(std::pair<float, float> target)
{
	float distance = distToPoint(X, Y, target.first, target.second);
	if (target == *(targetPositions->end()-1) && distance < 10.0f)
	{
		device->abortMacro();
		device->roll(0, 0, 0);
		return;
	}
	else
	{
		int angle = getAngle(target);
		device->abortMacro();
		device->roll((int)distance/2, angle, 1);
	}
}

void SpheroLogic::updateSpheroPos(float Xpos, float Ypos)
{
	X = Xpos/10.0f;
	Y = Ypos/10.0f;
}

int SpheroLogic::getAngle(std::pair<float, float> target)
{
	float dot = X * target.first + Y * target.second;
	float absDist = distToPoint(0.0f, 0.0f, target.first, target.second) * distToPoint(0.0f, 0.0f, X, Y);
	float cosAngle = dot / absDist;

	float angle = atan2(target.second - Y , target.first - X);
	//if (target.second < Y)
		//angle = -angle;
	angle = angle * (180.0f / 3.14f);
	
	//std::cout << "dot: " << dot << "	absDist: " << absDist << "	cosAngle: " << cosAngle << "	angle: " << angle;
	std::cout << "Angle:   " << angle << "     ";
	
	if (angle < 180.0f && angle > 90.0f)
		angle = 450.0f - angle;
	else
		angle = 90.0f - angle;



	std::cout << "SpheroAngle:     " << angle;
	angle = angle - offAngle;
	if (angle < 0)
		angle = 360 + angle;

	std::cout << "    offsettangle:  " << angle << std::endl;

	return int(angle);
}


