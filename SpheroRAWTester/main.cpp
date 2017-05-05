/**
 * Functionality based on:
 *
 * OpenCV SimpleBlobDetector Example
 *
 * Copyright 2015 by Satya Mallick <spmallick@gmail.com>
 *
 */

#include <iterator>
#include <iostream>
#include "opencv2/opencv.hpp"
//#include "Camera.h"
#include "../linearalgebra.hh"        //<-- Anton (windows)
//#include "../OpenCV/linearalgebra.hh"


using namespace cv;
using namespace std;


int readCalibrationData(Mat& cameraMatrix, Mat& extrinsicParam);


static void readSpheroCalibration(vector<double>& x3, vector<double>& reade0, vector<double>& reade1, a3d::Vector3d& e0, a3d::Vector3d& e1, bool& newbase);

static void saveSpheroPositions(vector<double> x3, vector<double> e0, vector<double> e1);


vector<double> operator-(vector<double> &vec1, vector<double> &vec2);

vector<double> operator/(vector<double> vec1, double d);

void setParams(SimpleBlobDetector::Params &params);

a3d::Vector3d calculateNormal(Mat cameraMatrix, Mat extrinsicParam);

void setCameraSettings(VideoCapture &capture);

void calculateNewBase(vector<double> x0, vector<double> x1, vector<double> x2, a3d::Vector3d &e0, a3d::Vector3d &e1);

//void operator=(vector<double> vec);

// Working blobdetection code, not class based
int main()
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
    vector<double> rv, bv, reade0, reade1;
        //vector<double> ge0, ge1;

 //   vector<double> nyBasPos; //vector att spara postitioner i
    vector<double> x0, x1, x2, x3; //, x1x0, x2x0,
    vector<double> nyBasPos;


    vector<KeyPoint> redkeypoints, bluekeypoints; // Storage for blob keypoints
    a3d::Vector3d v2r, v2b, transvec, transvec2, normal, redintersection, blueintersection;
    a3d::Vector3d e0 = {0.0, 0.0, 0.0}, e1 = {0.0, 0.0, 0.0};

    namedWindow("keypoints", WINDOW_NORMAL);
    namedWindow("red image", WINDOW_NORMAL);
    namedWindow("blue image", WINDOW_NORMAL);
    const Scalar RED(0,0,255), GREEN(0,255,0);

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
    double fx = cameraMatrix.at<double>(0,0);
    double cx = cameraMatrix.at<double>(0,2);
    double cy = cameraMatrix.at<double>(1,2);

    // separate the rotation and translation of extrinsic parameters
    // transvec as Vector3d, rotvec as vector<double>

    transvec = {extrinsicParam.at<double>(0,3), extrinsicParam.at<double>(0,4), extrinsicParam.at<double>(0,5)};

    //calculate normal
    normal = calculateNormal(cameraMatrix, extrinsicParam);

    //video capture object.
    VideoCapture capture(1);

    if(!capture.isOpened()){
        cout << "ERROR ACQUIRING VIDEO FEED\n";
        getchar();
        return -1;
    }

    //set camera settings so we can see our "glowing" spheros
    setCameraSettings(capture);

    // capture one frame and print image size
    capture >> frame;
    cout << "frame size: " << frame.size() << endl;

    while(true){

        // read frame
        capture >> frame;

        cvtColor(frame, imgHSV, COLOR_BGR2HSV); //Convert the captured frame from BGR to HSV

        // threshold red
        inRange(imgHSV, Scalar(redMinH, redMinS, redMinV), Scalar(redMaxH, redMaxS, redMaxV), redthresholdimg);
        //morphological closing (fill small holes in the foreground)
        dilate( redthresholdimg, redthresholdimg, getStructuringElement(MORPH_ELLIPSE, Size(5, 5)) );
        erode( redthresholdimg, redthresholdimg, getStructuringElement(MORPH_ELLIPSE, Size(5, 5)) );

        // threshold blue
        inRange(imgHSV, Scalar(blueMinH, blueMinS, blueMinV), Scalar(blueMaxH, blueMaxS, blueMaxV), bluethresholdimg);
        //morphological closing (fill small holes in the foreground)
        dilate( bluethresholdimg, bluethresholdimg, getStructuringElement(MORPH_ELLIPSE, Size(5, 5)) );
        erode( bluethresholdimg, bluethresholdimg, getStructuringElement(MORPH_ELLIPSE, Size(5, 5)) );

        // Detect red blobs
        detector->detect( redthresholdimg, redkeypoints );
        // Detect blue blobs
        detector->detect( bluethresholdimg, bluekeypoints );

        for(int i = 0; i < redkeypoints.size(); i++) {
            rX = redkeypoints[i].pt.x;
            rY = redkeypoints[i].pt.y;
        }
        for(int i = 0; i < bluekeypoints.size(); i++) {
            bX = bluekeypoints[i].pt.x;
            bY = bluekeypoints[i].pt.y;
        }

        // Calculate projection vector from camera
        rv = {((rX - cx)/fx), ((rY - cy)/fx), 1};
        bv = {((bX - cx)/fx), ((bY - cy)/fx), 1};

        // vector into vector3d
        for(int i = 0; i < rv.size() ; i++){
            v2r[i] = rv[i];     //red
        }

        for(int i = 0; i < bv.size() ; i++){
            v2b[i] = bv[i];     //blue
        }


        // calculate intersection of normal vector and projection vector
        // calculate d = length of line intersecting plane
        v2r = v2r.normalized();
        dr = (transvec * normal)/(v2r * normal);
        redintersection = dr * v2r;

        v2b = v2b.normalized();
        db = (transvec * normal)/(v2b * normal);
        blueintersection = db * v2b;

        //re-calibrate the new base if c is pressed
        char keyc = (char)waitKey(capture.isOpened() ? 50 : 1);
        if( capture.isOpened() && keyc == 'c' )
        {
            newbase = false;        //we no longer want this base

            while(!nyBasPos.empty())
            {
                nyBasPos.pop_back();
                newbasecounter--;
            }

            e0 = {1.1, 1.1, 1.1};
            e1 = {1.1, 1.1, 1.1};
            
        }

        // calculate positions in the new base if b is pressed
        char key = (char)waitKey(capture.isOpened() ? 50 : 1); //int riktigt säker på vad som ska stå istället för 1, stod delay förut
        if( capture.isOpened() && key == 'b' )
        {

            for(int k = 0; k < 2; k++) {
                nyBasPos.push_back(redintersection[k]);
                newbasecounter++;
                //j++;
            }
            cout << "newbasecounter: " << newbasecounter;
        }
        // only run this one time to calculate the new base
        if( !newbase && newbasecounter == 8 )
        {
            // x0,x1,x2 corner points, x3 middle point (= where origin is going to be)

            x0 = {nyBasPos[0], nyBasPos[1]};
            x1 = {nyBasPos[2], nyBasPos[3]};
            x2 = {nyBasPos[4], nyBasPos[5]};
            x3 = {nyBasPos[6], nyBasPos[7]};

            //calculate new base
            calculateNewBase(x0, x1, x2, e0, e1);

            // converting to vector<double> to be able to write to file
            reade0 = {e0[0], e0[1]};
            reade1 = {e1[0], e1[1]};
            newbase = true;             //we now have a new base

            saveSpheroPositions(x3, reade0, reade1);
        }

        //position in new coordinate base
        if(newbase){
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
        string msg = (newbase) ? "Calibrated" : "Press 'b' four times to calibrate";
        int baseLine = 0;
        Size textSize = getTextSize(msg, FONT_HERSHEY_PLAIN, 2, 2, &baseLine);
        Point textOrigin(frame.cols - 2*textSize.width - 10, frame.rows - 2*baseLine - 10);

        if(!newbase)
        {
            msg = format( "%d/%d", newbasecounter/2, 4);
        }
        putText( frame, msg, textOrigin, FONT_HERSHEY_PLAIN, 2, newbase ?  GREEN : RED, 2);

        cout << "red : " << redintersection;
        cout << "   Red Pos : " << redX << " , " << redY << "   Blue Pos : " << blueX << " , " << blueY<< endl;

        imshow("red image", redthresholdimg);
        imshow("blue image", bluethresholdimg);

        // Draw detected blobs as green circles (och blå).
        // DrawMatchesFlags::DRAW_RICH_KEYPOINTS flag ensures the size of the circle corresponds to the size of blob
        drawKeypoints( frame, redkeypoints, im_with_keypoints, Scalar(0,255,0), DrawMatchesFlags::DEFAULT );
        drawKeypoints( frame, bluekeypoints, im_with_keypoints, Scalar(0,0,255), DrawMatchesFlags::DEFAULT );

        // Show blobs
        imshow("keypoints", im_with_keypoints );
        waitKey(1);

        switch(waitKey(1)){

            case 27: //'esc' key has been pressed, exit program.
                return 0;
            case 112: //'p' has been pressed. this will pause/resume the code.
                pause = !pause;
                if(pause == true){ cout<<"Code paused, press 'p' again to resume"<<endl;
                    while (pause == true){
                        //stay in this loop until
                        switch (waitKey()){
                            //a switch statement inside a switch statement? Mind blown.
                            case 112:
                                //change pause back to false
                                pause = false;
                                cout<<"Code resumed."<<endl;
                                break;
                        }
                    }
                }
        }
    }
}

/**************************************************
 * Functions
 **************************************************/

// read camera matrix and extrinsics from XML-file
int readCalibrationData(Mat& cameraMatrix, Mat& extrinsicParam){

    // read camera matrix and extrinsic parameters from out_camera_data.xml
    const string calibrationFile = "../out_camera_data.xml";
    FileStorage fs(calibrationFile , FileStorage::READ);
    if (!fs.isOpened())
    {
        cerr << "failed to open " << calibrationFile << endl;
        return 1;
    }
    fs["camera_matrix"] >> cameraMatrix;
    fs["extrinsic_parameters"] >> extrinsicParam;
    fs.release();

    cout << "camera matrix : " << cameraMatrix << endl;
}


static void readSpheroCalibration(vector<double>& x3, vector<double>& reade0, vector<double>& reade1, a3d::Vector3d& e0, a3d::Vector3d& e1, bool& newbase)
{
    const string calibrationFile = "../base_config.xml";
    FileStorage fs(calibrationFile, FileStorage::READ); // Read the settings
    if (!fs.isOpened())
    {
        cout << "Could not open the configuration file: \"" << calibrationFile << "\"" << endl;
    }
    if(fs.isOpened())
    {
        fs["x3"] >> x3;
        fs["e0"] >> reade0;
        fs["e1"] >> reade1;
        e0 = {reade0[0], reade0[1], 0};
        e1 = {reade1[0], reade1[1], 0};
        newbase = true;
    }
    fs.release();
}


// Print sphero positions and base vectors to the output file
static void saveSpheroPositions(vector<double> x3, vector<double> e0, vector<double> e1)

{
    const string outputFileName = "../base_config.xml";
    FileStorage fs( outputFileName, FileStorage::WRITE );

    time_t tm;
    time( &tm );
    struct tm *t2 = localtime( &tm );
    char buf[1024];
    strftime( buf, sizeof(buf), "%c", t2 );

    fs << "calibration_time" << buf;

    fs << "x3" << x3;
    fs << "e0" << e0;
    fs << "e1" << e1;
    fs.release();

}

vector<double> operator-(vector<double> &vec1, vector<double> &vec2){
    return {vec1[0]-vec2[0], vec1[1]-vec2[1]};
}
vector<double> operator/(vector<double> vec1, double d){
    return {vec1[0]/d, vec1[1]/d};
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
    vector<double> rotvec;
    double value;
    Mat Rot(3,3, CV_32FC1);

    // separate the rotation and translation of extrinsic parameters
    // transvec as Vector3d, rotvec as vector<double>

    for( int i = 0; i < 3; i++)
    {
        value = extrinsicParam.at<double>(0,i);
        rotvec.push_back(value);
    }

    // Rodrigues transforms rotation vector from extrinsics to 3x3 array, stored in matrix Rot
    Rodrigues(rotvec, Rot, noArray());

    // invert y and z of Rot
    a3d::Matrix3d Mr(+Rot.at<double>(0,0), +Rot.at<double>(0,1), +Rot.at<double>(0,2),
                     -Rot.at<double>(1,0), -Rot.at<double>(1,1), -Rot.at<double>(1,2),
                     -Rot.at<double>(2,0), -Rot.at<double>(2,1), -Rot.at<double>(2,2));
    // quaternion rotation
    a3d::Quaterniond r = a3d::Quaterniond(Mr);
    cout << "quaternion r : " << r << endl;

    // rotate z with quaternion r
    normal = r.rotate(a3d::Vector3d(0,0,1));
    cout << "normal of plane : " << normal << endl;
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
    Contrast   = capture.get(CAP_PROP_CONTRAST );
    Saturation = capture.get(CAP_PROP_SATURATION);
    Gain       = capture.get(CAP_PROP_GAIN);
    exposure   = capture.get(CAP_PROP_EXPOSURE);

    cout<<"===================================="<<endl<<endl;
    cout<<"Default Brightness--------> "<<Brightness<<endl;
    cout<<"Default Contrast----------> "<<Contrast<<endl;
    cout<<"Default Saturation--------> "<<Saturation<<endl;
    cout<<"Default Gain--------------> "<<Gain<<endl<<endl;
    cout<<"Default Exposure----------> "<<exposure<<endl<<endl;
    cout<<"===================================="<<endl;
}

void calculateNewBase(vector<double> x0, vector<double> x1, vector<double> x2, a3d::Vector3d &e0, a3d::Vector3d &e1)
{
    vector<double>  x1x0, x2x0;
    vector<double> ge0, ge1;

    //if the positions happen to be the same, this won't give e0 and e1 null
    if( x0[0] == x1[0] && x0[1] == x1[1])
    {
        x0 = {1.1, 1.1, 1.1};
        x1 = {2.2, 2.2, 2.2};
    }

    if( x0[0] == x2[0] && x0[1] == x2[1])
    {
        x0 = {1.1, 1.1, 1.1};
        x2 = {3.3, 3.3, 3.3};
    }

    // x0,x1,x2 corner points, x3 middle point
    //x0 = {nyBasPos[0], nyBasPos[1]};
    //x1 = {nyBasPos[2], nyBasPos[3]};
    //x2 = {nyBasPos[4], nyBasPos[5]};
    //x3 = {nyBasPos[6], nyBasPos[7]};

    //x1 - x0
    x1x0 = {x1[0]-x0[0],x1[1]-x0[1]};
    //x2 - x0
    x2x0 = {x2[0]-x0[0],x2[1]-x0[1]};

    //new base, vector<double>
    ge0 = (x1-x0)/(sqrt((x1x0[0]*x1x0[0]+x1x0[1]*x1x0[1])));
    ge1 = (x2-x0)/(sqrt((x2x0[0]*x2x0[0]+x2x0[1]*x2x0[1])));

    //new base, Vector3d
    e0 = {ge0[0], ge0[1], 0};
    e1 = {ge1[0], ge1[1], 0};
}

/*
void operator=(vector<double> vec){
    this = {vec[0], vec[1], 0.0};
}*/

/* // [R|T] matrix with rotation and translation from extrinsic values
 *
cv::Mat componentMat ( transvec, true );
std::cout << componentMat << endl;
Mat RT(3,4, CV_32FC1);
hconcat(Rot, componentMat, RT);
cout << "[R|T] : " << RT << endl;
cout << "transvec: " <<  endl;
for(int i = 0; i < 3; i++)
{
    cout << transvec[i] << " ";
}*/

/*
// Class based main code, not working atm
// Reading camera parameters from XML to compute normal
int main( int argc, char** argv ) {
    const string calibrationFile = argc > 1 ? argv[1] : "../out_camera_data.xml";
    FileStorage fs(calibrationFile , FileStorage::READ);
    if (!fs.isOpened())
    {
        cerr << "failed to open " << calibrationFile << endl;
        return 1;
    }
    Mat cameraMatrix, distCoeffs, avgProjError, perViewReprojError, extrinsicParam, imagePoints;
    fs["camera_matrix"] >> cameraMatrix;
    fs["extrinsic_parameters"] >> extrinsicParam;
    fs.release();
    vector<double> rotvec, transvec;
    for( int i = 0; i < 3; i++)
    {
        double value = extrinsicParam.at<double>(0,i);
        rotvec.push_back(value);
    }
    for(int i = 3; i < 6; i++)
    {
       double value = extrinsicParam.at<double>(0,i);
       transvec.push_back(value);
    }
    Mat Rot(3,3, CV_32FC1);
    Rodrigues(rotvec, Rot, noArray());
    cv::Mat componentMat ( transvec, true );
    std::cout << componentMat << endl;
    Mat RT(3,4, CV_32FC1);
    hconcat(Rot, componentMat, RT);
    cout << "[R|T] : " << RT << endl;
    cout << "transvec: " <<  endl;
    for(int i = 0; i < 3; i++)
    {
        cout << transvec[i] << " ";
    }
    a3d::Matrix3d Mr(+Rot.at<double>(0,0), +Rot.at<double>(0,1), +Rot.at<double>(0,2),
                     -Rot.at<double>(1,0), -Rot.at<double>(1,1), -Rot.at<double>(1,2),
                     -Rot.at<double>(2,0), -Rot.at<double>(2,1), -Rot.at<double>(2,2));
    a3d::Quaterniond r = a3d::Quaterniond(Mr);
    cout << "quaternion r : " << r << endl;
    a3d::Vector3d normal = r.rotate(a3d::Vector3d(0,0,1));
    cout << "normal of plane : " << normal << endl;
    //a3d::Vector3d floor_normal = r * a3d::Vector3d(0,0,1);
    return 0;
    WebCam cam1;
    vector<KeyPoint> pos;
    while(true) {
        // store sphero pos in vector
        cam1.updateCamera();
        // save keypoints
        Mat im_with_keypoints;
        // Draw detected blobs as red circles.
        // DrawMatchesFlags::DRAW_RICH_KEYPOINTS flag ensures
        // the size of the circle corresponds to the size of blob
        drawKeypoints( cam1.frame, cam1.keypoints, im_with_keypoints, Scalar(0,0,255), DrawMatchesFlags::DRAW_RICH_KEYPOINTS );
        // Show blobs
        namedWindow("keypoints", 0 );
        imshow("keypoints", im_with_keypoints );
        //resizeWindow("keypoints", 1920, 1080);
        //imshow("binary image", binarizedImage);
        waitKey(1);
        bool pause = false;
        switch (waitKey(10)) {
            case 27: //'esc' key has been pressed, exit program.
                //delete cam1;
                return 0;
            case 112: //'p' has been pressed. this will pause/resume the code.
                pause = !pause;
                if (pause == true) {
                    cout << "Code paused, press 'p' again to resume" << endl;
                    while (pause == true) {
                        //stay in this loop until
                        switch (waitKey()) {
                            //a switch statement inside a switch statement? Mind blown.
                            case 112:
                                //change pause back to false
                                pause = false;
                                cout << "Code resumed." << endl;
                                break;
                        }
                    }
                }
        }
    }
}*/

// Find HSV color values
/*
#include <sstream>
#include <string>
#include <iostream>
#include "opencv/highgui.h"
#include "opencv/cv.h"
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/core/core.hpp>
#include <opencv2/opencv.hpp>
#include <iostream>
#include "opencv2/highgui/highgui.hpp"
#include "opencv2/imgproc/imgproc.hpp"
using namespace cv;
using namespace std;
//initial min and max HSV filter values.
//these will be changed using trackbars
int H_MIN = 0;
int H_MAX = 256;
int S_MIN = 0;
int S_MAX = 256;
int V_MIN = 0;
int V_MAX = 256;
//default capture width and height
const int FRAME_WIDTH = 640;
const int FRAME_HEIGHT = 480;
//max number of objects to be detected in frame
const int MAX_NUM_OBJECTS = 50;
//minimum and maximum object area
const int MIN_OBJECT_AREA = 20 * 20;
const int MAX_OBJECT_AREA = FRAME_HEIGHT*FRAME_WIDTH / 1.5;
//names that will appear at the top of each window
const string windowName = "Original Image";
const string windowName1 = "HSV Image";
const string windowName2 = "Thresholded Image";
const string windowName3 = "After Morphological Operations";
const string trackbarWindowName = "Trackbars";
bool calibrationMode;//used for showing debugging windows, trackbars etc.
bool mouseIsDragging;//used for showing a rectangle on screen as user clicks and drags mouse
bool mouseMove;
bool rectangleSelected;
Point initialClickPoint, currentMousePoint; //keep track of initial point clicked and current position of mouse
Rect rectangleROI; //this is the ROI that the user has selected
vector<int> H_ROI, S_ROI, V_ROI;// HSV values from the click/drag ROI region stored in separate vectors so that we can sort them easily
void on_trackbar(int, void*)
{//This function gets called whenever a
    // trackbar position is changed
    //for now, this does nothing.
}
void createTrackbars(){
    //create window for trackbars
    namedWindow(trackbarWindowName, 0);
    //create memory to store trackbar name on window
    char TrackbarName[50];
    sprintf(TrackbarName, "H_MIN", H_MIN);
    sprintf(TrackbarName, "H_MAX", H_MAX);
    sprintf(TrackbarName, "S_MIN", S_MIN);
    sprintf(TrackbarName, "S_MAX", S_MAX);
    sprintf(TrackbarName, "V_MIN", V_MIN);
    sprintf(TrackbarName, "V_MAX", V_MAX);
    //create trackbars and insert them into window
    //3 parameters are: the address of the variable that is changing when the trackbar is moved(eg.H_LOW),
    //the max value the trackbar can move (eg. H_HIGH),
    //and the function that is called whenever the trackbar is moved(eg. on_trackbar)
    //                                  ---->    ---->     ---->
    createTrackbar("H_MIN", trackbarWindowName, &H_MIN, 255, on_trackbar);
    createTrackbar("H_MAX", trackbarWindowName, &H_MAX, 255, on_trackbar);
    createTrackbar("S_MIN", trackbarWindowName, &S_MIN, 255, on_trackbar);
    createTrackbar("S_MAX", trackbarWindowName, &S_MAX, 255, on_trackbar);
    createTrackbar("V_MIN", trackbarWindowName, &V_MIN, 255, on_trackbar);
    createTrackbar("V_MAX", trackbarWindowName, &V_MAX, 255, on_trackbar);
}
void clickAndDrag_Rectangle(int event, int x, int y, int flags, void* param){
    //only if calibration mode is true will we use the mouse to change HSV values
    if (calibrationMode == true){
        //get handle to video feed passed in as "param" and cast as Mat pointer
        Mat* videoFeed = (Mat*)param;
        if (event == CV_EVENT_LBUTTONDOWN && mouseIsDragging == false)
        {
            //keep track of initial point clicked
            initialClickPoint = cv::Point(x, y);
            //user has begun dragging the mouse
            mouseIsDragging = true;
        }
        // user is dragging the mouse
        if (event == CV_EVENT_MOUSEMOVE && mouseIsDragging == true)
        {
            //keep track of current mouse point
            currentMousePoint = cv::Point(x, y);
            //user has moved the mouse while clicking and dragging
            mouseMove = true;
        }
        // user has released left button
        if (event == CV_EVENT_LBUTTONUP && mouseIsDragging == true)
        {
            //set rectangle ROI to the rectangle that the user has selected
            rectangleROI = Rect(initialClickPoint, currentMousePoint);
            //reset boolean variables
            mouseIsDragging = false;
            mouseMove = false;
            rectangleSelected = true;
        }
        if (event == CV_EVENT_RBUTTONDOWN){
            //user has clicked right mouse button
            //Reset HSV Values
            H_MIN = 0;
            S_MIN = 0;
            V_MIN = 0;
            H_MAX = 255;
            S_MAX = 255;
            V_MAX = 255;
        }
        if (event == CV_EVENT_MBUTTONDOWN){
            //user has clicked middle mouse button
            //enter code here if needed.
        }
    }
}
void recordHSV_Values(cv::Mat frame, cv::Mat hsv_frame){
    //save HSV values for ROI that user selected to a vector
    if (mouseMove == false && rectangleSelected == true){
        //clear previous vector values
        if (H_ROI.size()>0) H_ROI.clear();
        if (S_ROI.size()>0) S_ROI.clear();
        if (V_ROI.size()>0 )V_ROI.clear();
        //if the rectangle has no width or height (user has only dragged a line) then we don't try to iterate over the width or height
        if (rectangleROI.width<1 || rectangleROI.height<1) cout << "Please drag a rectangle, not a line" << endl;
        else{
            for (int i = rectangleROI.x; i<rectangleROI.x + rectangleROI.width; i++){
                //iterate through both x and y direction and save HSV values at each and every point
                for (int j = rectangleROI.y; j<rectangleROI.y + rectangleROI.height; j++){
                    //save HSV value at this point
                    H_ROI.push_back((int)hsv_frame.at<cv::Vec3b>(j, i)[0]);
                    S_ROI.push_back((int)hsv_frame.at<cv::Vec3b>(j, i)[1]);
                    V_ROI.push_back((int)hsv_frame.at<cv::Vec3b>(j, i)[2]);
                }
            }
        }
        //reset rectangleSelected so user can select another region if necessary
        rectangleSelected = false;
        //set min and max HSV values from min and max elements of each array
        if (H_ROI.size()>0){
            //NOTE: min_element and max_element return iterators so we must dereference them with "*"
            H_MIN = *std::min_element(H_ROI.begin(), H_ROI.end());
            H_MAX = *std::max_element(H_ROI.begin(), H_ROI.end());
            cout << "MIN 'H' VALUE: " << H_MIN << endl;
            cout << "MAX 'H' VALUE: " << H_MAX << endl;
        }
        if (S_ROI.size()>0){
            S_MIN = *std::min_element(S_ROI.begin(), S_ROI.end());
            S_MAX = *std::max_element(S_ROI.begin(), S_ROI.end());
            cout << "MIN 'S' VALUE: " << S_MIN << endl;
            cout << "MAX 'S' VALUE: " << S_MAX << endl;
        }
        if (V_ROI.size()>0){
            V_MIN = *std::min_element(V_ROI.begin(), V_ROI.end());
            V_MAX = *std::max_element(V_ROI.begin(), V_ROI.end());
            cout << "MIN 'V' VALUE: " << V_MIN << endl;
            cout << "MAX 'V' VALUE: " << V_MAX << endl;
        }
    }
    if (mouseMove == true){
        //if the mouse is held down, we will draw the click and dragged rectangle to the screen
        rectangle(frame, initialClickPoint, cv::Point(currentMousePoint.x, currentMousePoint.y), cv::Scalar(0, 255, 0), 1, 8, 0);
    }
}
string intToString(int number){
    std::stringstream ss;
    ss << number;
    return ss.str();
}
void drawObject(int x, int y, Mat &frame){
    //use some of the openCV drawing functions to draw crosshairs
    //on your tracked image!
    //'if' and 'else' statements to prevent
    //memory errors from writing off the screen (ie. (-25,-25) is not within the window)
    circle(frame, Point(x, y), 20, Scalar(0, 255, 0), 2);
    if (y - 25>0)
        line(frame, Point(x, y), Point(x, y - 25), Scalar(0, 255, 0), 2);
    else
        line(frame, Point(x, y), Point(x, 0), Scalar(0, 255, 0), 2);
    if (y + 25<FRAME_HEIGHT)
        line(frame, Point(x, y), Point(x, y + 25), Scalar(0, 255, 0), 2);
    else
        line(frame, Point(x, y), Point(x, FRAME_HEIGHT), Scalar(0, 255, 0), 2);
    if (x - 25>0)
        line(frame, Point(x, y), Point(x - 25, y), Scalar(0, 255, 0), 2);
    else
        line(frame, Point(x, y), Point(0, y), Scalar(0, 255, 0), 2);
    if (x + 25<FRAME_WIDTH)
        line(frame, Point(x, y), Point(x + 25, y), Scalar(0, 255, 0), 2);
    else
        line(frame, Point(x, y), Point(FRAME_WIDTH, y), Scalar(0, 255, 0), 2);
    putText(frame, intToString(x) + "," + intToString(y), Point(x, y + 30), 1, 1, Scalar(0, 255, 0), 2);
}
void morphOps(Mat &thresh){
    //create structuring element that will be used to "dilate" and "erode" image.
    //the element chosen here is a 3px by 3px rectangle
    Mat erodeElement = getStructuringElement(MORPH_RECT, Size(3, 3));
    //dilate with larger element so make sure object is nicely visible
    Mat dilateElement = getStructuringElement(MORPH_RECT, Size(8, 8));
    erode(thresh, thresh, erodeElement);
    erode(thresh, thresh, erodeElement);
    dilate(thresh, thresh, dilateElement);
    dilate(thresh, thresh, dilateElement);
}
void trackFilteredObject(int &x, int &y, Mat threshold, Mat &cameraFeed){
    Mat temp;
    threshold.copyTo(temp);
    //these two vectors needed for output of findContours
    vector< vector<Point> > contours;
    vector<Vec4i> hierarchy;
    //find contours of filtered image using openCV findContours function
    findContours(temp, contours, hierarchy, CV_RETR_CCOMP, CV_CHAIN_APPROX_SIMPLE);
    //use moments method to find our filtered object
    double refArea = 0;
    int largestIndex = 0;
    bool objectFound = false;
    if (hierarchy.size() > 0) {
        int numObjects = hierarchy.size();
        //if number of objects greater than MAX_NUM_OBJECTS we have a noisy filter
        if (numObjects<MAX_NUM_OBJECTS){
            for (int index = 0; index >= 0; index = hierarchy[index][0]) {
                Moments moment = moments((cv::Mat)contours[index]);
                double area = moment.m00;
                //if the area is less than 20 px by 20px then it is probably just noise
                //if the area is the same as the 3/2 of the image size, probably just a bad filter
                //we only want the object with the largest area so we save a reference area each
                //iteration and compare it to the area in the next iteration.
                if (area>MIN_OBJECT_AREA && area<MAX_OBJECT_AREA && area>refArea){
                    x = moment.m10 / area;
                    y = moment.m01 / area;
                    objectFound = true;
                    refArea = area;
                    //save index of largest contour to use with drawContours
                    largestIndex = index;
                }
                else objectFound = false;
            }
            //let user know you found an object
            if (objectFound == true){
                putText(cameraFeed, "Tracking Object", Point(0, 50), 2, 1, Scalar(0, 255, 0), 2);
                //draw object location on screen
                drawObject(x, y, cameraFeed);
                //draw largest contour
                //drawContours(cameraFeed, contours, largestIndex, Scalar(0, 255, 255), 2);
            }
        }
        else putText(cameraFeed, "TOO MUCH NOISE! ADJUST FILTER", Point(0, 50), 1, 2, Scalar(0, 0, 255), 2);
    }
}
int main(int argc, char* argv[])
{
    //some boolean variables for different functionality within this
    //program
    bool trackObjects = true;
    bool useMorphOps = true;
    calibrationMode = true;
    //Matrix to store each frame of the webcam feed
    Mat cameraFeed;
    //matrix storage for HSV image
    Mat HSV;
    //matrix storage for binary threshold image
    Mat threshold;
    //x and y values for the location of the object
    int x = 0, y = 0;
    //video capture object to acquire webcam feed
    VideoCapture capture;
    //open capture object at location zero (default location for webcam)
    capture.open(1);
    // camera properties
    capture.set(CAP_PROP_FRAME_WIDTH, 1920);
    capture.set(CAP_PROP_FRAME_HEIGHT, 1080);
    capture.set(CAP_PROP_GAIN, 0.0);
    capture.set(CAP_PROP_SATURATION, 255.0);
    capture.set(CAP_PROP_BRIGHTNESS, 100.0);
    capture.set(CAP_PROP_EXPOSURE, -5.0);
    capture.set(CAP_PROP_CONTRAST, 255.0);
    //set height and width of capture frame
    capture.set(CV_CAP_PROP_FRAME_WIDTH, FRAME_WIDTH);
    capture.set(CV_CAP_PROP_FRAME_HEIGHT, FRAME_HEIGHT);
    //must create a window before setting mouse callback
    cv::namedWindow(windowName);
    //set mouse callback function to be active on "Webcam Feed" window
    //we pass the handle to our "frame" matrix so that we can draw a rectangle to it
    //as the user clicks and drags the mouse
    cv::setMouseCallback(windowName, clickAndDrag_Rectangle, &cameraFeed);
    //initiate mouse move and drag to false
    mouseIsDragging = false;
    mouseMove = false;
    rectangleSelected = false;
    //start an infinite loop where webcam feed is copied to cameraFeed matrix
    //all of our operations will be performed within this loop
    while (1){
        //store image to matrix
        capture.read(cameraFeed);
        //convert frame from BGR to HSV colorspace
        cvtColor(cameraFeed, HSV, COLOR_BGR2HSV);
        //set HSV values from user selected region
        recordHSV_Values(cameraFeed, HSV);
        //filter HSV image between values and store filtered image to
        //threshold matrix
        inRange(HSV, Scalar(H_MIN, S_MIN, V_MIN), Scalar(H_MAX, S_MAX, V_MAX), threshold);
        //perform morphological operations on thresholded image to eliminate noise
        //and emphasize the filtered object(s)
        if (useMorphOps)
            morphOps(threshold);
        //pass in thresholded frame to our object tracking function
        //this function will return the x and y coordinates of the
        //filtered object
        if (trackObjects)
            trackFilteredObject(x, y, threshold, cameraFeed);
        //show frames
        if (calibrationMode == true){
            //create slider bars for HSV filtering
            createTrackbars();
            imshow(windowName1, HSV);
            imshow(windowName2, threshold);
        }
        else{
            destroyWindow(windowName1);
            destroyWindow(windowName2);
            destroyWindow(trackbarWindowName);
        }
        imshow(windowName, cameraFeed);
        //delay 30ms so that screen can refresh.
        //image will not appear without this waitKey() command
        //also use waitKey command to capture keyboard input
        if (waitKey(30) == 99) calibrationMode = !calibrationMode;//if user presses 'c', toggle calibration mode
    }
    return 0;
}*/