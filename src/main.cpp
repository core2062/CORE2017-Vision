#include "cscore.h"
#include "networktables/NetworkTable.h"
#include "opencv2/imgproc.hpp"
#include "opencv2/highgui.hpp"
#include "COREVisionLib.h"
#include "llvm/StringRef.h"

#include <iostream>
#include <unistd.h>
#include <chrono>

//#define NOTRASPI
#define USEGEARVISION
#define BLURGEAR
#define DILATEGEAR

using namespace std;
using namespace CORE;
using namespace cs;
using namespace cv;
using namespace llvm;

const StringRef ipAddress = "10.20.62.63";
const StringRef gearCameraName = "gear";
const StringRef frontCameraName = "front";

const int gearCameraPort = 0;
const int visionCameraPort = 1;

const int streamResolution[2] = {416, 240};
const int visionResolution[2] = {1280, 720};

string selectedCamera;

/*CvSource nullSource("Null Camera", VideoMode::kMJPEG, streamResolution[0], streamResolution[1], 60);
CvSource visionCameraSource("Vision Camera", VideoMode::kMJPEG, visionResolution[0], visionResolution[1], 60);

void updateNullCamera(string message) {
    Mat nothing(streamResolution[1], streamResolution[0], CV_8UC3, Scalar(67, 54,244));
    putText(nothing, message, cvPoint(30,30), FONT_HERSHEY_SIMPLEX, 0.5, cvScalar(255,255,255), 2);
    nullSource.PutFrame(nothing);
}*/

UsbCamera gearCamera("Gear Camera", gearCameraPort);
UsbCamera visionCamera("Vision Camera", visionCameraPort);
CvSink *gearCameraSink;
CvSink *visionCameraSink;
CvSource gearStreamSource("Gear Camera Streaming Source", VideoMode::kMJPEG,
                          streamResolution[0], streamResolution[1], 30);
CvSource visionStreamSource("Vision Camera Streaming Source", VideoMode::kMJPEG,
                            visionResolution[0], visionResolution[1], 30);
MjpegServer mjpegServer("HTTP Server", 5800);

shared_ptr<NetworkTable> visionTable;

Mat originalGearCameraFrame, BWGearFrame;

#ifdef USEGEARVISION
Mat gearThresholdedImage;
vector<vector<Point>> gearContours;
vector<Vec4i> gearHierarchy;
#endif

void processGearCameraFrame(uint64_t time) {
    if(time == 0) {
        cout << "Error in getting gear frame: " << /*gearCameraSink->GetError() <<*/ endl;
        return;
    }
    gearCameraSink->GrabFrame(originalGearCameraFrame);
    if(originalGearCameraFrame.empty()) {
        cout << "Error in getting gear frame!" << endl;
        return;
    }
    if(visionTable->ContainsKey("enableGearVision") && visionTable->GetBoolean("enableGearVision", false)) {
        boxFilter(originalGearCameraFrame, originalGearCameraFrame, -1, Size(50, 50), Point(-1,-1), true,
                  BORDER_CONSTANT);

        //inRange(originalGearCameraFrame, Scalar(0, 84, 117), Scalar(72, 255, 255), gearThresholdedImage);
        inRange(originalGearCameraFrame, Scalar(0, 142, 169), Scalar(139, 255, 255), gearThresholdedImage);
        gearContours.clear();
        gearHierarchy.clear();
        findContours(gearContours, gearContours, gearHierarchy, CV_RETR_EXTERNAL, CV_CHAIN_APPROX_SIMPLE, Point(0, 0));
        int largestSize = 0;
        Point center;
        for(auto contour: gearContours) {
            if(contourArea(contour) > largestSize) {
                auto rect = boundingRect(contour);
                center.x = (rect.tl().x + (rect.width * 0.5));
                center.y = ((rect.tl().y + (rect.height * 0.5)));
            }
        }
        center.y = streamResolution[1] - center.y;
        visionTable->PutNumber("targetX", center.x);
        visionTable->PutNumber("targetY", center.y);
    }
    if(selectedCamera == gearCameraName) {
        cvtColor(originalGearCameraFrame, BWGearFrame, COLOR_RGB2GRAY);
        gearStreamSource.PutFrame(BWGearFrame);
        mjpegServer.SetSource(gearStreamSource);

    }
}

Mat originalVisionCameraFrame, BWVisionFrame, visionThresholdedImage;
vector<vector<Point>> visionContours;
vector<Rect> visionBoundingBoxes;
vector<Vec4i> visionHierarchy;
const double targetWidthToHeight = 2.0/5.0;

CORETimer deltaTime;
int frameCount = 0;

double image = 1;

void processVisionCameraFrame(uint64_t time) {
    if(time == 0) {
        cout << "Error in getting vision frame: " << /*visionCameraSink->GetError() <<*/ endl;
        return;
    }
    time = visionCameraSink->GrabFrame(originalVisionCameraFrame);
    /*originalVisionCameraFrame = imread("../images/practiceImages/" + to_string((int)image) + "_1280x720.jpg", IMREAD_COLOR);
    image += 0.5;*/
    if(originalVisionCameraFrame.empty()) {
        cout << "Error in getting vision frame!" << endl;
        return;
    }
    cvtColor(originalVisionCameraFrame, BWVisionFrame, COLOR_RGB2GRAY);
    threshold(BWVisionFrame, visionThresholdedImage, 55, 255, ThresholdTypes::THRESH_BINARY);
    visionContours.clear();
    visionHierarchy.clear();
    findContours(visionThresholdedImage, visionContours, visionHierarchy, CV_RETR_EXTERNAL, CV_CHAIN_APPROX_SIMPLE, Point(0, 0) );
    visionBoundingBoxes.clear();
    for(auto contour : visionContours) {
        Rect boundingBox = boundingRect(contour);
        if((abs((boundingBox.width / boundingBox.height) - targetWidthToHeight) < 0.5)
           && (abs((boundingBox.width * boundingBox.height) - contourArea(contour)) < 4000)
           && (boundingBox.width * boundingBox.height) > 200) {
            visionBoundingBoxes.push_back(boundingBox);
        }
    }
    vector<Rect> topTwoBoundingBoxes;
    for(int i = 0; i < 2; i++) {
        Rect maxBoundingBox;
        int currentMaxArea = 0;
        for(int j = 0; j < visionBoundingBoxes.size(); j++) {
            int area = visionBoundingBoxes[j].width * visionBoundingBoxes[j].height;
            if(area > currentMaxArea) {
                maxBoundingBox = visionBoundingBoxes[j];
                currentMaxArea = area;
                visionBoundingBoxes.erase(visionBoundingBoxes.begin() + j, visionBoundingBoxes.begin() + j+1);
            }
        }
        topTwoBoundingBoxes.push_back(maxBoundingBox);
    }
    Point center;
    if(!topTwoBoundingBoxes.empty()) {
        center.x = 0;
        center.y = 0;
        int i = 0;
        for(auto boundingBox : topTwoBoundingBoxes) {
            center.x += (boundingBox.tl().x + (boundingBox.width * 0.5));
            center.y += ((boundingBox.tl().y + (boundingBox.height * 0.5)));
            i++;
        }
        if(i > 0) {
            center.x /= i;
            center.y /= i;
        }
        if(selectedCamera == frontCameraName) {
            circle(originalVisionCameraFrame, center, 3, Scalar(0, 255, 0), 5, 2);
        }
        center.y = visionResolution[1] - center.y;
    } else {
        center.x = -1;
        center.y = -1;
    }
    //cout << "X: " << center.x << " Y: " << center.y << endl;
    visionTable->PutNumber("piTime", time);
    visionTable->PutNumber("targetX", center.x);
    visionTable->PutNumber("targetY", center.y);
    //center.y = visionResolution[1];
    if(selectedCamera == frontCameraName) {
        for(int i = 0; i < topTwoBoundingBoxes.size() && i < 2; i++) {
            rectangle(originalVisionCameraFrame, topTwoBoundingBoxes[i], Scalar(0, 0, 255), 3);
        }
        visionStreamSource.PutFrame(originalVisionCameraFrame);
        mjpegServer.SetSource(visionStreamSource);
    }
    double delta = deltaTime.Get();
    if(delta >= 1) {
        double fps = ((double)frameCount) / delta;
        cout << "FPS: " << fps << endl;
        visionTable->PutNumber("fps", fps);
        frameCount = 0;
        deltaTime.Reset();
        deltaTime.Start();
    } else {
        frameCount++;
    }
}

int main() {
    NetworkTable::SetClientMode();
    NetworkTable::SetTeam(2062);
    NetworkTable::Initialize();
    visionTable = NetworkTable::GetTable("Vision");

    gearCamera.SetVideoMode(VideoMode::kMJPEG, streamResolution[0], streamResolution[1], 30);
    visionCamera.SetVideoMode(VideoMode::kMJPEG, visionResolution[0], visionResolution[1], 90);

    /*if(visionCamera.GetName().find("LifeCam") != string::npos) {
        swap(gearCamera, visionCamera);
    }*/
    gearCameraSink = new CvSink("Gear Camera Processing", processGearCameraFrame);
    visionCameraSink= new CvSink("Vision Camera Processing", processVisionCameraFrame);

    gearCameraSink->SetSource(gearCamera);
    visionCameraSink->SetSource(visionCamera);

    gearCameraSink->SetEnabled(true);
    visionCameraSink->SetEnabled(true);

    mjpegServer.SetSource(gearCamera);
    selectedCamera = gearCameraName;

    deltaTime.Reset();
    deltaTime.Start();

#ifdef NOTRASPI
    Mat frame;
    namedWindow("original");
    namedWindow("processed");
#endif
    usleep(100000);
    while(true) {
        //processGearCameraFrame(1);
        if(visionTable->ContainsKey("disablePegVision") && visionTable->GetBoolean("disablePegVision", false)) {

        } else {
            processVisionCameraFrame(1);
        }

        if(visionTable->ContainsKey("disableGearVision") && visionTable->GetBoolean("disablePegVision", false)) {

        } else {
            processVisionCameraFrame(1);
        }

        //Camera streaming and switching
        if(visionTable->ContainsKey("camera")) {
            selectedCamera = visionTable->GetString("camera", "gear");
        } else {
            selectedCamera = "gear";
        }

        if(selectedCamera == gearCameraName) {
            mjpegServer.SetSource(gearCamera);
        }
#ifdef NOTRASPI
        visionCameraSink->GrabFrame(frame);
        imshow("original", frame);
        imshow("processed", BWVisionFrame);
        waitKey(1);
#endif
    }
}

/*

 */