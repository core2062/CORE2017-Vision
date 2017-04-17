#include "cscore.h"
#include "networktables/NetworkTable.h"
#include "opencv2/imgproc.hpp"
#include "opencv2/highgui.hpp"
#include "COREVisionLib.h"
#include "llvm/StringRef.h"

#include <iostream>
#include <unistd.h>
#include <chrono>

#define USEGEARVISION
#define BLURGEAR
#define MANUALIMAGEMODE

using namespace std;
using namespace CORE;
using namespace cs;
using namespace cv;
using namespace llvm;

const StringRef gearCameraName = "gear";
const StringRef backCameraName = "front";

const int gearCameraPort = 0;
const int backCameraPort = 1;
const int visionCameraPort = 2;

const int streamResolution[2] = {416, 240};
const int visionResolution[2] = {640, 360};

string selectedCamera;

UsbCamera gearCamera("Gear Camera", gearCameraPort);
UsbCamera backCamera("Back Camera", backCameraPort);
UsbCamera visionCamera("Vision Camera", visionCameraPort);
CvSink *gearCameraSink;
CvSink *backCameraSink;
CvSink *visionCameraSink;
CvSource gearStreamSource("Gear Camera Streaming Source", VideoMode::kMJPEG,
                          streamResolution[0], streamResolution[1], 30);
CvSource backStreamSource("Back Camera Streaming Source", VideoMode::kMJPEG,
                          streamResolution[0], streamResolution[1], 30);
CvSource visionStreamSource("Vision Debug Streaming Source", VideoMode::kMJPEG,
                            visionResolution[0], visionResolution[1], 30);
CvSource backDebugSource("Back Camera Debug Source", VideoMode::kMJPEG,
                          streamResolution[0], streamResolution[1], 30);
MjpegServer driverVisionServer("Driver Vision Server", 5800);
MjpegServer visionDebugServer("Vision Debug Server", 5801);

shared_ptr<NetworkTable> visionTable;

Mat originalGearCameraFrame, BWGearFrame;

#ifdef USEGEARVISION
Mat gearThresholdedImage;
vector<vector<Point>> gearContours;
vector<Vec4i> gearHierarchy;
#endif

double gearImage = 1;

void processGearCameraFrame(uint64_t time) {
    gearCameraSink->GrabFrame(originalGearCameraFrame);
#ifdef MANUALIMAGEMODE
/*    originalGearCameraFrame = imread("../gearImages/" + to_string((int)gearImage) + "_640x360.jpg",
                                     IMREAD_COLOR);
    gearImage += 0.1;*/
#endif
    if(originalGearCameraFrame.empty() || time == 0) {
        cout << "Error in getting gear frame!" << endl;
        return;
    }
    if(visionTable->ContainsKey("enableGearVision") && visionTable->GetBoolean("enableGearVision", false)) {
        //Gear vision processing
        boxFilter(originalGearCameraFrame, originalGearCameraFrame, -1, Size(50, 50), Point(-1,-1), true,
                  BORDER_CONSTANT);
        inRange(originalGearCameraFrame, Scalar(0, 142, 169), Scalar(139, 255, 255), gearThresholdedImage);
        gearContours.clear();
        gearHierarchy.clear();
        findContours(gearThresholdedImage, gearContours, gearHierarchy, CV_RETR_EXTERNAL, CV_CHAIN_APPROX_SIMPLE,
                     Point(0, 0));
        int largestSize = 0;
        Point center;
        center.x = -1;
        center.y = -1;
        for(auto contour: gearContours) {
            if(contourArea(contour) > largestSize) {
                auto rect = boundingRect(contour);
                center.x = (int)(rect.tl().x + (rect.width * 0.5));
                center.y = (int)((rect.tl().y + (rect.height * 0.5)));
            }
        }
        circle(originalGearCameraFrame, center, 3, Scalar(0, 255, 0), 5, 2);
        if(center.y != -1) {
            center.y = streamResolution[1] - center.y;
        }
        visionTable->PutNumber("targetX", center.x);
        visionTable->PutNumber("targetY", center.y);
        backDebugSource.PutFrame(originalGearCameraFrame);
    }
    if(selectedCamera == gearCameraName) {
        cvtColor(originalGearCameraFrame, BWGearFrame, COLOR_RGB2GRAY);
        gearStreamSource.PutFrame(BWGearFrame);
        driverVisionServer.SetSource(gearStreamSource);
    }
}

Mat originalBackCameraFrame, BWBackFrame;

void backCameraFrame(uint64_t time) {
    backCameraSink->GrabFrame(originalBackCameraFrame);
    if(originalBackCameraFrame.empty() || time == 0) {
        cout << "Error in getting back frame!" << endl;
        return;
    }
    if(selectedCamera == backCameraName) {
        cvtColor(originalBackCameraFrame, BWBackFrame, COLOR_RGB2GRAY);
        backStreamSource.PutFrame(BWBackFrame);
        driverVisionServer.SetSource(backStreamSource);
    }
}

Mat originalVisionCameraFrame, BWVisionFrame, visionThresholdedImage;
vector<vector<Point>> visionContours;
vector<Rect> visionBoundingBoxes;
vector<Vec4i> visionHierarchy;
const double targetWidthToHeight = 2.0/5.0;

CORETimer deltaTime;
int frameCount = 0;

double visionImage = 1;

void processVisionCameraFrame(uint64_t time) {
    time = visionCameraSink->GrabFrame(originalVisionCameraFrame);
#ifdef MANUALIMAGEMODE
    originalVisionCameraFrame = imread("../pegImages/practiceImages/" + to_string((int)visionImage) + "_640x360.jpg",
                                       IMREAD_COLOR);
    visionImage += 0.1;
#endif
    if(originalVisionCameraFrame.empty()/* || time == 0*/) {
        cout << "Error in getting vision frame!" << endl;
        return;
    }
    cvtColor(originalVisionCameraFrame, BWVisionFrame, COLOR_RGB2GRAY);
    threshold(BWVisionFrame, visionThresholdedImage, 50, 255, ThresholdTypes::THRESH_BINARY); //55
    visionContours.clear();
    visionHierarchy.clear();
    findContours(visionThresholdedImage, visionContours, visionHierarchy, CV_RETR_EXTERNAL, CV_CHAIN_APPROX_SIMPLE,
                 Point(0, 0));
    visionBoundingBoxes.clear();
    for(auto contour : visionContours) {
        Rect boundingBox = boundingRect(contour);
        if((abs((boundingBox.width / boundingBox.height) - targetWidthToHeight) < 0.5)
           && (abs((boundingBox.width * boundingBox.height) - contourArea(contour)) < 2000)
           && (boundingBox.width * boundingBox.height) > 1000) {
            visionBoundingBoxes.push_back(boundingBox);
        }
    }
    vector<Rect> topTwoBoundingBoxes;
    for(int i = 0; i < 2; i++) {
        if(!visionBoundingBoxes.empty()) {
            Rect maxBoundingBox;
            int currentMaxArea = 0;
            for(int j = 0; j < visionBoundingBoxes.size(); j++) {
                int area = visionBoundingBoxes[j].width * visionBoundingBoxes[j].height;
                if(area > currentMaxArea) {
                    maxBoundingBox = visionBoundingBoxes[j];
                    currentMaxArea = area;
                    visionBoundingBoxes.erase(visionBoundingBoxes.begin() + j, visionBoundingBoxes.begin() + j + 1);
                }
            }
            topTwoBoundingBoxes.push_back(maxBoundingBox);
        }
    }
    Point center;
    if(!topTwoBoundingBoxes.empty()) {
        center.x = 0;
        center.y = 0;
        for(auto boundingBox : topTwoBoundingBoxes) {
            center.x += (boundingBox.tl().x + (boundingBox.width * 0.5));
            center.y += ((boundingBox.tl().y + (boundingBox.height * 0.5)));
        }
        center.x /= topTwoBoundingBoxes.size();
        center.y /= topTwoBoundingBoxes.size();
        if(selectedCamera == backCameraName) {
            circle(originalVisionCameraFrame, center, 3, Scalar(0, 255, 0), 5, 2);
        }
        center.y = visionResolution[1] - center.y;
    } else {
        center.x = -1;
        center.y = -1;
    }
    visionTable->PutNumber("piTime", time);
    visionTable->PutNumber("targetX", center.x);
    visionTable->PutNumber("targetY", center.y);
    if(selectedCamera == backCameraName) {
        for(int i = 0; i < topTwoBoundingBoxes.size() && i < 2; i++) {
            rectangle(originalVisionCameraFrame, topTwoBoundingBoxes[i], Scalar(0, 0, 255), 3);
        }
        visionStreamSource.PutFrame(originalVisionCameraFrame);
        visionDebugServer.SetSource(visionStreamSource);
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
    backCamera.SetVideoMode(VideoMode::kMJPEG, streamResolution[0], streamResolution[1], 30);
    visionCamera.SetVideoMode(VideoMode::kMJPEG, visionResolution[0], visionResolution[1], 90);

    gearCameraSink = new CvSink("Gear Camera Processing", processGearCameraFrame);
    backCameraSink = new CvSink("Back Camera Processing", backCameraFrame);
    visionCameraSink = new CvSink("Vision Camera Processing", processVisionCameraFrame);

    gearCameraSink->SetSource(gearCamera);
    backCameraSink->SetSource(backCamera);
    visionCameraSink->SetSource(visionCamera);

    gearCameraSink->SetEnabled(true); //Probably not necessary
    backCameraSink->SetEnabled(true);
    visionCameraSink->SetEnabled(true);

    driverVisionServer.SetSource(gearCamera);
    selectedCamera = gearCameraName;

    visionDebugServer.SetSource(gearCamera);

    deltaTime.Reset();
    deltaTime.Start();

    while(true) {
        selectedCamera = backCameraName; //TODO: FIX THIS
        if(visionTable->ContainsKey("disablePegVision") && visionTable->GetBoolean("disablePegVision", false)) {

        } else {
            processVisionCameraFrame(1);
        }

        if(visionTable->ContainsKey("disableGearVision") && visionTable->GetBoolean("disableGearVision", false)) {

        } else {
            processGearCameraFrame(1);
        }

        if(visionTable->ContainsKey("camera")) {
            selectedCamera = visionTable->GetString("camera", "gear");
        } else {
            selectedCamera = "gear";
        }
        driverVisionServer.SetSource(gearStreamSource); //TODO: FIX THIS
        visionDebugServer.SetSource(visionStreamSource);
    }
}