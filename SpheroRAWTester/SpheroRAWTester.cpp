//######################################################################################################################
/*
    Copyright (c) since 2014 - Paul Freund

    Permission is hereby granted, free of charge, to any person
    obtaining a copy of this software and associated documentation
    files (the "Software"), to deal in the Software without
    restriction, including without limitation the rights to use,
    copy, modify, merge, publish, distribute, sublicense, and/or sell
    copies of the Software, and to permit persons to whom the
    Software is furnished to do so, subject to the following
    conditions:

    The above copyright notice and this permission notice shall be
    included in all copies or substantial portions of the Software.

    THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND,
    EXPRESS OR IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES
    OF MERCHANTABILITY, FITNESS FOR A PARTICULAR PURPOSE AND
    NONINFRINGEMENT. IN NO EVENT SHALL THE AUTHORS OR COPYRIGHT
    HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER LIABILITY,
    WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING
    FROM, OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR
    OTHER DEALINGS IN THE SOFTWARE.
*/
//######################################################################################################################

//Allows unsecure func(fopen and localtime)
#define _CRT_SECURE_NO_DEPRECATE

#include "stdafx.h"
#include <windows.h>
#include "SpheroRAWItf.h"
#include <msclr\marshal_cppstd.h>

#include <string>
#include <iostream>
#include "SpheroLogic.h"

#include <stdio.h>
#include <iterator>
#include "opencv2/opencv.hpp"
//#include "Camera.h"
#include "linearalgebra.hh"        //<-- Anton (windows)

#using <System.dll>

using namespace System;
using namespace System::IO;
using namespace System::Net;
using namespace System::Net::Sockets;
using namespace System::Text;
using namespace System::Threading;
//using namespace std;
using namespace cv;

void startServer();

void startCameraTracking();

int readCalibrationData(Mat& cameraMatrix, Mat& extrinsicParam);

static void readSpheroCalibration(std::vector<double>& x3, std::vector<double>& reade0, std::vector<double>& reade1, a3d::Vector3d& e0, a3d::Vector3d& e1, bool& newbase);

static void saveSpheroPositions(std::vector<double> x3, std::vector<double> e0, std::vector<double> e1);

std::vector<double> operator-(std::vector<double> &vec1, std::vector<double> &vec2);

std::vector<double> operator/(std::vector<double> vec1, double d);

void setParams(SimpleBlobDetector::Params &params);

a3d::Vector3d calculateNormal(Mat cameraMatrix, Mat extrinsicParam);

void setCameraSettings(VideoCapture &capture);

void calculateNewBase(std::vector<double> x0, std::vector<double> x1, std::vector<double> x2, a3d::Vector3d &e0, a3d::Vector3d &e1);

std::string response;
bool gotMessage = false;
float Xpos, Ypos;
//======================================================================================================================

void PrintDeviceStatus(std::string action, ISpheroDevice* device) {
    std::cout << "Action: " << action << " Result: ";

    if(device == nullptr) {
        std::cout << "Error - Sphero handle is invalid" << std::endl;
        return;
    }

    switch(device->state()) {
        case SpheroState_None:                          { std::cout << "SpheroRAW not initialized"                   << std::endl; break; }
        case SpheroState_Error_BluetoothError:          { std::cout << "Error - Couldn't initialize Bluetooth stack" << std::endl; break; }
        case SpheroState_Error_BluetoothUnavailable:    { std::cout << "Error - No valid Bluetooth adapter found"    << std::endl; break; }
        case SpheroState_Error_NotPaired:               { std::cout << "Error - Specified Sphero not Paired"         << std::endl; break; }
        case SpheroState_Error_ConnectionFailed:        { std::cout << "Error - Connecting failed"                   << std::endl; break; }
        case SpheroState_Disconnected:                  { std::cout << "Sphero disconnected"                         << std::endl; break; }
        case SpheroState_Connected:                     { std::cout << "Sphero connected"                            << std::endl; break; }
    }

	std::cout << std::endl;
}

void OrbBasicAppendLine(ISpheroDevice* device, std::string lineText) {
    std::vector<ubyte> program(lineText.begin(), lineText.end());
    program.insert(program.end(), '\n');
    device->appendOrbBasicFragment(0, program);
}

//======================================================================================================================

int _tmain(int argc, _TCHAR* argv[])
{
    //------------------------------------------------------------------------------------------------------------------
    // Create device 
	SpheroLogic^ sphero1 = gcnew SpheroLogic("Sphero-WRG");
    bool quit = false;

	Thread^ serverThread = gcnew Thread(gcnew ThreadStart(startServer));
	serverThread->Start();

	Thread^ cameraThread = gcnew Thread(gcnew ThreadStart(startCameraTracking));
	cameraThread->Start();

    while(!quit) {
        //------------------------------------------------------------------------------------------------------------------
        // Connect 
        //------------------------------------------------------------------------------------------------------------------
        // Send/Receive Data

		sphero1->setOrientation();

        while(sphero1->spheroConnected()) {

			if (gotMessage)
			{
				sphero1->setTarget(response);
				Thread^ spheroThread = gcnew Thread(gcnew ThreadStart(sphero1, &SpheroLogic::moveSphero));
				spheroThread->Start();
				//sphero1->testMove();
				gotMessage = false;
			}
			sphero1->updateSpheroPos(Xpos, Ypos);
			Sleep(100);


			if (GetAsyncKeyState('Q')) {
				quit = true;
				break;
			}
        //sphero1->printDeviceStatus("Poll loop exited");

        }
        Sleep(100);
    }

    //------------------------------------------------------------------------------------------------------------------
    // Disconnect 
    sphero1->printDeviceStatus("Disconnect");

    //------------------------------------------------------------------------------------------------------------------
    // Destroy device 
	sphero1->~SpheroLogic();

    //------------------------------------------------------------------------------------------------------------------
    // Keep terminal open 
    std::cin.get();
	return 0;
}

/**************************************************
* Functions
**************************************************/

// read camera matrix and extrinsics from XML-file
int readCalibrationData(Mat& cameraMatrix, Mat& extrinsicParam) {

	// read camera matrix and extrinsic parameters from out_camera_data.xml
	const std::string calibrationFile = "out_camera_data.xml";
	FileStorage fs(calibrationFile, FileStorage::READ);
	if (!fs.isOpened())
	{
		std::cerr << "failed to open " << calibrationFile << std::endl;
		return 1;
	}
	fs["camera_matrix"] >> cameraMatrix;
	fs["extrinsic_parameters"] >> extrinsicParam;
	fs.release();

	std::cout << "camera matrix : " << cameraMatrix << std::endl;
}


static void readSpheroCalibration(std::vector<double>& x3, std::vector<double>& reade0, std::vector<double>& reade1, a3d::Vector3d& e0, a3d::Vector3d& e1, bool& newbase)
{
	const std::string calibrationFile = "../base_config.xml";
	FileStorage fs(calibrationFile, FileStorage::READ); // Read the settings
	if (!fs.isOpened())
	{
		std::cout << "Could not open the configuration file: \"" << calibrationFile << "\"" << std::endl;
	}
	if (fs.isOpened())
	{
		fs["x3"] >> x3;
		fs["e0"] >> reade0;
		fs["e1"] >> reade1;
		e0 = { reade0[0], reade0[1], 0 };
		e1 = { reade1[0], reade1[1], 0 };
		newbase = true;
	}
	fs.release();
}


// Print sphero positions and base vectors to the output file
static void saveSpheroPositions(std::vector<double> x3, std::vector<double> e0, std::vector<double> e1)

{
	const std::string outputFileName = "../base_config.xml";
	FileStorage fs(outputFileName, FileStorage::WRITE);

	time_t tm;
	time(&tm);
	struct tm *t2 = localtime(&tm);
	char buf[1024];
	strftime(buf, sizeof(buf), "%c", t2);

	fs << "calibration_time" << buf;

	fs << "x3" << x3;
	fs << "e0" << e0;
	fs << "e1" << e1;
	fs.release();

}

std::vector<double> operator-(std::vector<double> &vec1, std::vector<double> &vec2) {
	return{ vec1[0] - vec2[0], vec1[1] - vec2[1] };
}
std::vector<double> operator/(std::vector<double> vec1, double d) {
	return{ vec1[0] / d, vec1[1] / d };
}

void setParams(SimpleBlobDetector::Params &params)
{
	// frame thresholds
	params.minThreshold = 50;
	params.maxThreshold = 250;
	params.thresholdStep = 50;
	// Filter by color
	params.filterByColor = false;
	// Filter by Area.
	params.filterByArea = true;
	params.minArea = 100;
	params.maxArea = 10000;
	// Filter by Circularity
	params.filterByCircularity = true;
	params.minCircularity = 0.1;
	// Filter by Convexity
	params.filterByConvexity = false;
	// Filter by Inertia
	params.filterByInertia = false;
}

a3d::Vector3d calculateNormal(Mat cameraMatrix, Mat extrinsicParam)
{
	a3d::Vector3d transvec, normal;
	std::vector<double> rotvec;
	double value;
	Mat Rot(3, 3, CV_32FC1);

	// separate the rotation and translation of extrinsic parameters
	// transvec as Vector3d, rotvec as vector<double>

	for (int i = 0; i < 3; i++)
	{
		value = extrinsicParam.at<double>(0, i);
		rotvec.push_back(value);
	}

	// Rodrigues transforms rotation vector from extrinsics to 3x3 array, stored in matrix Rot
	Rodrigues(rotvec, Rot, noArray());

	// invert y and z of Rot
	a3d::Matrix3d Mr(+Rot.at<double>(0, 0), +Rot.at<double>(0, 1), +Rot.at<double>(0, 2),
		-Rot.at<double>(1, 0), -Rot.at<double>(1, 1), -Rot.at<double>(1, 2),
		-Rot.at<double>(2, 0), -Rot.at<double>(2, 1), -Rot.at<double>(2, 2));
	// quaternion rotation
	a3d::Quaterniond r = a3d::Quaterniond(Mr);
	std::cout << "quaternion r : " << r << std::endl;

	// rotate z with quaternion r
	normal = r.rotate(a3d::Vector3d(0, 0, 1));
	std::cout << "normal of plane : " << normal << std::endl;
	return normal;
}

void setCameraSettings(VideoCapture &capture)
{
	double Brightness, exposure, Contrast, Saturation, Gain;

	// Det här funkar bra som fan!
	capture.set(CAP_PROP_FRAME_WIDTH, 1280);
	capture.set(CAP_PROP_FRAME_HEIGHT, 720);
	capture.set(CAP_PROP_GAIN, 0.0);
	capture.set(CAP_PROP_SATURATION, 255.0);
	capture.set(CAP_PROP_BRIGHTNESS, 100.0);
	capture.set(CAP_PROP_EXPOSURE, -5.0);
	capture.set(CAP_PROP_CONTRAST, 255.0);

	Brightness = capture.get(CAP_PROP_BRIGHTNESS);
	Contrast = capture.get(CAP_PROP_CONTRAST);
	Saturation = capture.get(CAP_PROP_SATURATION);
	Gain = capture.get(CAP_PROP_GAIN);
	exposure = capture.get(CAP_PROP_EXPOSURE);

	std::cout << "====================================" << std::endl << std::endl;
	std::cout << "Default Brightness--------> " << Brightness << std::endl;
	std::cout << "Default Contrast----------> " << Contrast << std::endl;
	std::cout << "Default Saturation--------> " << Saturation << std::endl;
	std::cout << "Default Gain--------------> " << Gain << std::endl << std::endl;
	std::cout << "Default Exposure----------> " << exposure << std::endl << std::endl;
	std::cout << "====================================" << std::endl;
}

void calculateNewBase(std::vector<double> x0, std::vector<double> x1, std::vector<double> x2, a3d::Vector3d &e0, a3d::Vector3d &e1)
{
	std::vector<double>  x1x0, x2x0;
	std::vector<double> ge0, ge1;

	//if the positions happen to be the same, this won't give e0 and e1 null
	if (x0[0] == x1[0] && x0[1] == x1[1])
	{
		x0 = { 1.1, 1.1, 1.1 };
		x1 = { 2.2, 2.2, 2.2 };
	}

	if (x0[0] == x2[0] && x0[1] == x2[1])
	{
		x0 = { 1.1, 1.1, 1.1 };
		x2 = { 3.3, 3.3, 3.3 };
	}

	// x0,x1,x2 corner points, x3 middle point
	//x0 = {nyBasPos[0], nyBasPos[1]};
	//x1 = {nyBasPos[2], nyBasPos[3]};
	//x2 = {nyBasPos[4], nyBasPos[5]};
	//x3 = {nyBasPos[6], nyBasPos[7]};

	//x1 - x0
	x1x0 = { x1[0] - x0[0],x1[1] - x0[1] };
	//x2 - x0
	x2x0 = { x2[0] - x0[0],x2[1] - x0[1] };

	//new base, vector<double>
	ge0 = (x1 - x0) / (sqrt((x1x0[0] * x1x0[0] + x1x0[1] * x1x0[1])));
	ge1 = (x2 - x0) / (sqrt((x2x0[0] * x2x0[0] + x2x0[1] * x2x0[1])));

	//new base, Vector3d
	e0 = { ge0[0], ge0[1], 0 };
	e1 = { ge1[0], ge1[1], 0 };
}

void startCameraTracking()
{
	/* Constants and Variables definitions */

	bool newbase = false, pause = false;
	//double value;
	//double Brightness, exposure, Contrast, Saturation, Gain,
	double redX = 0.0, redY = 0.0, blueX = 0.0, blueY = 0.0;
	double dr, db;
	float rX = 0, rY = 0, bX = 0, bY = 0; //
	int newbasecounter = 0, j = 0;
	Mat cameraMatrix, extrinsicParam; // camera calibration data
									  //Mat Rot(3,3, CV_32FC1); // for Rodrigues vector->matrix transform
	Mat frame; // video capture container
	Mat im_with_keypoints; // keypoints container
	Mat imgHSV, redthresholdimg, bluethresholdimg;

	//vector<double> rotvec;// transvec;
	std::vector<double> rv, bv, reade0, reade1;
	//vector<double> ge0, ge1;

	//   vector<double> nyBasPos; //vector att spara postitioner i
	std::vector<double> x0, x1, x2, x3; //, x1x0, x2x0,
	std::vector<double> nyBasPos;


	std::vector<KeyPoint> redkeypoints, bluekeypoints; // Storage for blob keypoints
	a3d::Vector3d v2r, v2b, transvec, transvec2, normal, redintersection, blueintersection;
	a3d::Vector3d e0 = { 0.0, 0.0, 0.0 }, e1 = { 0.0, 0.0, 0.0 };

	namedWindow("keypoints", WINDOW_NORMAL);
	namedWindow("red image", WINDOW_NORMAL);
	namedWindow("blue image", WINDOW_NORMAL);
	const Scalar RED(0, 0, 255), GREEN(0, 255, 0);

	// HSV color-space threshold values
	int redMinH = 0;
	int redMaxH = 20;
	int redMinS = 220;
	int redMaxS = 255;
	int redMinV = 200;
	int redMaxV = 240;

	int blueMinH = 80;
	int blueMaxH = 100;
	int blueMinS = 230;
	int blueMaxS = 255;
	int blueMinV = 210;
	int blueMaxV = 250;

	// Setup SimpleBlobDetector parameters.
	SimpleBlobDetector::Params params;

	//set blobtracking-params
	setParams(params);

	// Set up detector with params
	Ptr<SimpleBlobDetector> detector = SimpleBlobDetector::create(params);

	//read camera calibration data
	readCalibrationData(cameraMatrix, extrinsicParam);

	// Try to read Sphero calibration data points
	readSpheroCalibration(x3, reade0, reade1, e0, e1, newbase);

	// camera intrinsic parameter fx ( fx == fy ), calibrated at fullHD 1920x1080
	double fx = cameraMatrix.at<double>(0, 0);
	double cx = cameraMatrix.at<double>(0, 2);
	double cy = cameraMatrix.at<double>(1, 2);

	// separate the rotation and translation of extrinsic parameters
	// transvec as Vector3d, rotvec as vector<double>

	transvec = { extrinsicParam.at<double>(0,3), extrinsicParam.at<double>(0,4), extrinsicParam.at<double>(0,5) };

	//calculate normal
	normal = calculateNormal(cameraMatrix, extrinsicParam);

	//video capture object.
	VideoCapture capture(2);

	if (!capture.isOpened()) {
		std::cout << "ERROR ACQUIRING VIDEO FEED\n";
		getchar();
		return;
	}

	//set camera settings so we can see our "glowing" spheros
	setCameraSettings(capture);

	// capture one frame and print image size
	capture >> frame;
	std::cout << "frame size: " << frame.size() << std::endl;

	while (true) {

		// read frame
		capture >> frame;

		cvtColor(frame, imgHSV, COLOR_BGR2HSV); //Convert the captured frame from BGR to HSV

												// threshold red
		inRange(imgHSV, Scalar(redMinH, redMinS, redMinV), Scalar(redMaxH, redMaxS, redMaxV), redthresholdimg);
		//morphological closing (fill small holes in the foreground)
		dilate(redthresholdimg, redthresholdimg, getStructuringElement(MORPH_ELLIPSE, Size(5, 5)));
		erode(redthresholdimg, redthresholdimg, getStructuringElement(MORPH_ELLIPSE, Size(5, 5)));

		// threshold blue
		inRange(imgHSV, Scalar(blueMinH, blueMinS, blueMinV), Scalar(blueMaxH, blueMaxS, blueMaxV), bluethresholdimg);
		//morphological closing (fill small holes in the foreground)
		dilate(bluethresholdimg, bluethresholdimg, getStructuringElement(MORPH_ELLIPSE, Size(5, 5)));
		erode(bluethresholdimg, bluethresholdimg, getStructuringElement(MORPH_ELLIPSE, Size(5, 5)));

		// Detect red blobs
		detector->detect(redthresholdimg, redkeypoints);
		// Detect blue blobs
		detector->detect(bluethresholdimg, bluekeypoints);

		for (int i = 0; i < redkeypoints.size(); i++) {
			rX = redkeypoints[i].pt.x;
			rY = redkeypoints[i].pt.y;
		}
		for (int i = 0; i < bluekeypoints.size(); i++) {
			bX = bluekeypoints[i].pt.x;
			bY = bluekeypoints[i].pt.y;
		}

		// Calculate projection vector from camera
		rv = { ((rX - cx) / fx), ((rY - cy) / fx), 1 };
		bv = { ((bX - cx) / fx), ((bY - cy) / fx), 1 };

		// vector into vector3d
		for (int i = 0; i < rv.size(); i++) {
			v2r[i] = rv[i];     //red
		}

		for (int i = 0; i < bv.size(); i++) {
			v2b[i] = bv[i];     //blue
		}


		// calculate intersection of normal vector and projection vector
		// calculate d = length of line intersecting plane
		v2r = v2r.normalized();
		dr = (transvec * normal) / (v2r * normal);
		redintersection = dr * v2r;

		v2b = v2b.normalized();
		db = (transvec * normal) / (v2b * normal);
		blueintersection = db * v2b;

		//re-calibrate the new base if c is pressed
		char keyc = (char)waitKey(capture.isOpened() ? 50 : 1);
		if (capture.isOpened() && keyc == 'c')
		{
			newbase = false;        //we no longer want this base

			while (!nyBasPos.empty())
			{
				nyBasPos.pop_back();
				newbasecounter--;
			}

			e0 = { 1.1, 1.1, 1.1 };
			e1 = { 1.1, 1.1, 1.1 };

		}

		// calculate positions in the new base if b is pressed
		char key = (char)waitKey(capture.isOpened() ? 50 : 1); //int riktigt säker på vad som ska stå istället för 1, stod delay förut
		if (capture.isOpened() && key == 'b')
		{

			for (int k = 0; k < 2; k++) {
				nyBasPos.push_back(redintersection[k]);
				newbasecounter++;
				//j++;
			}
			//std::cout << "newbasecounter: " << newbasecounter;
		}
		// only run this one time to calculate the new base
		if (!newbase && newbasecounter == 8)
		{
			// x0,x1,x2 corner points, x3 middle point (= where origin is going to be)

			x0 = { nyBasPos[0], nyBasPos[1] };
			x1 = { nyBasPos[2], nyBasPos[3] };
			x2 = { nyBasPos[4], nyBasPos[5] };
			x3 = { nyBasPos[6], nyBasPos[7] };

			//calculate new base
			calculateNewBase(x0, x1, x2, e0, e1);

			// converting to vector<double> to be able to write to file
			reade0 = { e0[0], e0[1] };
			reade1 = { e1[0], e1[1] };
			newbase = true;             //we now have a new base

			saveSpheroPositions(x3, reade0, reade1);
		}

		//position in new coordinate base
		if (newbase) {
			// coordinates with origin in middle of AR marker
			redintersection[0] -= x3[0];
			redintersection[1] -= x3[1];
			redX = e0*redintersection;
			redY = e1*redintersection;

			blueintersection[0] -= x3[0];
			blueintersection[1] -= x3[1];
			blueX = e0*blueintersection;
			blueY = e1*blueintersection;
		}

		//----------------------------- Output Text ------------------------------------------------
		//! [output_text]
		std::string msg = (newbase) ? "Calibrated" : "Press 'b' four times to calibrate";
		int baseLine = 0;
		Size textSize = getTextSize(msg, FONT_HERSHEY_PLAIN, 2, 2, &baseLine);
		Point textOrigin(frame.cols - 2 * textSize.width - 10, frame.rows - 2 * baseLine - 10);

		if (!newbase)
		{
			msg = format("%d/%d", newbasecounter / 2, 4);
		}
		putText(frame, msg, textOrigin, FONT_HERSHEY_PLAIN, 2, newbase ? GREEN : RED, 2);

		//std::cout << "red : " << redintersection;
		//std::cout << "   Red Pos : " << redX << " , " << redY << "   Blue Pos : " << blueX << " , " << blueY << std::endl;
		Xpos = (float)redX;
		Ypos = (float)redY;

		imshow("red image", redthresholdimg);
		imshow("blue image", bluethresholdimg);

		// Draw detected blobs as green circles (och blå).
		// DrawMatchesFlags::DRAW_RICH_KEYPOINTS flag ensures the size of the circle corresponds to the size of blob
		drawKeypoints(frame, redkeypoints, im_with_keypoints, Scalar(0, 255, 0), DrawMatchesFlags::DEFAULT);
		drawKeypoints(frame, bluekeypoints, im_with_keypoints, Scalar(0, 0, 255), DrawMatchesFlags::DEFAULT);

		// Show blobs
		imshow("keypoints", im_with_keypoints);
		waitKey(1);

		switch (waitKey(1)) {

		case 27: //'esc' key has been pressed, exit program.
			return;
		case 112: //'p' has been pressed. this will pause/resume the code.
			pause = !pause;
			if (pause == true) {
				std::cout << "Code paused, press 'p' again to resume" << std::endl;
				while (pause == true) {
					//stay in this loop until
					switch (waitKey()) {
						//a switch statement inside a switch statement? Mind blown.
					case 112:
						//change pause back to false
						pause = false;
						std::cout << "Code resumed." << std::endl;
						break;
					}
				}
			}
		}
	}
}

void startServer()
{
	try
	{

		// Set the TcpListener on port 13000.
		Int32 port = 6321;
		IPAddress^ localAddr = IPAddress::Parse("192.168.43.236");

		// TcpListener* server = new TcpListener(port);
		TcpListener^ server = gcnew TcpListener(localAddr, port);

		// Start listening for client requests.
		server->Start();

		// Buffer for reading data
		cli::array<Byte>^bytes = gcnew cli::array<Byte>(256);
		System::String^ data = nullptr;

		// Enter the listening loop.
		while (true)
		{
			Console::Write("Waiting for a connection... \n");

			// Perform a blocking call to accept requests.
			// You could also user server.AcceptSocket() here.
			TcpClient^ client = server->AcceptTcpClient();
			Console::WriteLine("Connected!\n");
			data = nullptr;

			// Get a stream Object* for reading and writing
			NetworkStream^ stream = client->GetStream();
			Int32 i;

			// Loop to receive all the data sent by the client.
			while (i = stream->Read(bytes, 0, bytes->Length))
			{

				// Translate data bytes to a ASCII String*.
				data = Text::Encoding::ASCII->GetString(bytes, 0, i);
				Console::WriteLine("Received: {0}", data);
				response = msclr::interop::marshal_as<std::string>(data);
				gotMessage = true;

				/*
				// Process the data sent by the client.
				data = data->ToUpper();
				cli::array<Byte>^msg = Text::Encoding::ASCII->GetBytes(data);

				// Send back a response.
				stream->Write(msg, 0, msg->Length);
				Console::WriteLine("Sent: {0}", data);
				*/
			}

			// Shutdown and end connection
			client->Close();
		}
	}
	catch (SocketException^ e)
	{
		Console::WriteLine("SocketException: {0}", e);
	}

	Console::WriteLine("\nHit enter to continue...");
	Console::Read();
}
//======================================================================================================================
