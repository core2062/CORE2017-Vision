#include "cscore.h"
#include "networktables/NetworkTable.h"
#include "opencv2/imgproc.hpp"
#include "opencv2/highgui.hpp"
#include "COREVisionLib.h"
#include "llvm/StringRef.h"

#include <iostream>
#include <unistd.h>
#include <chrono>

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

CvSource visionDebugSource("Vision Debug Streaming Source", VideoMode::kMJPEG,
                           visionResolution[0], visionResolution[1], 30);
CvSource gearDebugSource("Back Camera Debug Source", VideoMode::kMJPEG,
                         streamResolution[0], streamResolution[1], 30);
MjpegServer driverVisionServer("Driver Vision Server", 5800);
MjpegServer visionDebugServer("Vision Debug Server", 5801);

Mat originalGearCameraFrame, BWGearFrame;

Mat gearThresholdedImage;
vector<vector<Point>> gearContours;
vector<Vec4i> gearHierarchy;

double gearImage = 1;

void processGearCameraFrame(Mat frame, double timestamp, shared_ptr<NetworkTable> visionTable) {
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
    if(selectedCamera == gearCameraName) {
        circle(originalGearCameraFrame, center, 3, Scalar(0, 255, 0), 5, 2);
    }
    if(center.y != -1) {
        center.y = streamResolution[1] - center.y;
    }
    visionTable->PutNumber("gearX", center.x);
    visionTable->PutNumber("gearY", center.y);
    if(selectedCamera == gearCameraName) {
        gearDebugSource.PutFrame(originalGearCameraFrame);
        visionDebugServer.SetSource(gearDebugSource);
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

void processVisionCameraFrame(Mat frame, double timestamp, shared_ptr<NetworkTable> visionTable) {
    cvtColor(originalVisionCameraFrame, BWVisionFrame, COLOR_RGB2GRAY);
    threshold(BWVisionFrame, visionThresholdedImage, 45, 255, ThresholdTypes::THRESH_BINARY); //55
    visionContours.clear();
    visionHierarchy.clear();
    findContours(visionThresholdedImage, visionContours, visionHierarchy, CV_RETR_EXTERNAL, CV_CHAIN_APPROX_SIMPLE,
                 Point(0, 0));
    visionBoundingBoxes.clear();
    for(auto contour : visionContours) {
        Rect boundingBox = boundingRect(contour);
        if((abs((boundingBox.width / boundingBox.height) - targetWidthToHeight) < 0.5)
           && (abs((boundingBox.width * boundingBox.height) - contourArea(contour)) < 1600)
           && (boundingBox.width * boundingBox.height) > 800) {
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
    if(topTwoBoundingBoxes.size() > 1) {
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
    visionTable->PutNumber("piTime", timestamp);
    visionTable->PutNumber("targetX", center.x);
    visionTable->PutNumber("targetY", center.y);
    if(selectedCamera == backCameraName) {
        for(int i = 0; i < topTwoBoundingBoxes.size() && i < 2; i++) {
            rectangle(originalVisionCameraFrame, topTwoBoundingBoxes[i], Scalar(0, 0, 255), 3);
        }
        visionDebugSource.PutFrame(originalVisionCameraFrame);
        visionDebugServer.SetSource(visionDebugSource);
        driverVisionServer.SetSource(visionDebugSource);
    }
}
