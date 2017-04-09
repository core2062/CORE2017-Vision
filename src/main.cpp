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

using namespace std;
using namespace CORE;
using namespace cs;
using namespace cv;
using namespace llvm;

const StringRef ipAddress = "10.20.62.63";
const StringRef gearCameraName = "gear";
const StringRef frontCameraName = "front";

const int gearCameraPort = 1;
const int frontCameraPort = 2;

const int streamResolution[2] = {240, 416};
const int visionResolution[2] = {720, 1280};

const double targetWidthToHeight = 2.0/5.0;

CvSource * nullSource, * visionCameraSource;

void updateNullCamera(string message) {
    Mat nothing(streamResolution[0], streamResolution[1], CV_8UC3, Scalar(67, 54,244));
    putText(nothing, message, cvPoint(30,30), FONT_HERSHEY_SIMPLEX, 0.5, cvScalar(255,255,255), 2);
    nullSource->PutFrame(nothing);
}

int main() {
    CORETimer timeout;
    shared_ptr<NetworkTable> visionTable;

    UsbCamera gearCamera("Gear Camera", gearCameraPort);
    gearCamera.SetVideoMode(VideoMode::kMJPEG, streamResolution[0], streamResolution[1], 30);

    visionCameraSource = new CvSource("Vision Camera", VideoMode::kMJPEG, visionResolution[0], visionResolution[1], 30);
    nullSource = new CvSource("Null Camera", VideoMode::kMJPEG, streamResolution[0], streamResolution[1], 30);

    CORECapture visionCamera(0, 1280, 720, false, true);
    visionCamera.SetFileInputLocation("../images/practiceImages/1.jpg", MULTI_IMAGE);

    visionCamera.SetBrightness(0);
    visionCamera.WaitForNewFrame(true);

    //Gear Camera
    timeout.Reset();
    timeout.Start();
    while(!gearCamera.IsConnected()) {
        cout << "Waiting for gear camera" << endl;
        usleep(100000); //0.1 seconds
        if(timeout.Get() > 1) {
            cout << "Error! Could not initialize gear camera!" << endl;
            break;
        }
    }

    NetworkTable::SetClientMode();
    NetworkTable::SetIPAddress(ipAddress);
    NetworkTable::Initialize();
    visionTable = NetworkTable::GetTable("Vision");

    MjpegServer mjpegServer("httpserver", 5800);
    mjpegServer.SetSource(gearCamera);

#ifdef NOTRASPI
    namedWindow( "Display window", WINDOW_AUTOSIZE );
    namedWindow( "Display window2", WINDOW_AUTOSIZE );
#endif
    Mat original, BWImage, thresholdedImage;
    vector<vector<Point>> contours;
    vector<Rect> boundingBoxes;
    vector<Vec4i> hierarchy;
    int image = 1;

    while(true) {
#ifdef NOTRASPI
            visionCamera.UseFileInput(true);
            visionCamera.SetFileInputLocation("../images/practiceImages/" + to_string(image) + ".jpg", MULTI_IMAGE);
#endif
            auto original = visionCamera.GetFrame();
            cvtColor(original.first, BWImage, COLOR_RGB2GRAY);
            threshold(BWImage, thresholdedImage, 55, 255, ThresholdTypes::THRESH_BINARY);
            contours.clear();
            hierarchy.clear();
            findContours(thresholdedImage, contours, hierarchy, CV_RETR_TREE, CV_CHAIN_APPROX_SIMPLE, Point(0, 0) );
            boundingBoxes.clear();
            for(auto contour : contours) {
                Rect boundingBox = boundingRect(contour);
                if((abs((boundingBox.width / boundingBox.height) - targetWidthToHeight) < 0.5)
                   && (abs((boundingBox.width * boundingBox.height) - contourArea(contour)) < 500)) {
                    boundingBoxes.push_back(boundingBox);
                }
                //cout << "X: " << boundingBox.x << " Y: " << boundingBox.y << endl;
            }
            vector<Rect> topTwoBoundingBoxes;
            for(int i = 0; i < 2; i++) {
                Rect maxBoundingBox;
                int currentMaxArea = 0;
                for(int j = 0; j < boundingBoxes.size(); j++) {
                    int area = boundingBoxes[j].width * boundingBoxes[j].height;
                    if(area > currentMaxArea) {
                        maxBoundingBox = boundingBoxes[j];
                        currentMaxArea = area;
                        boundingBoxes.erase(boundingBoxes.begin() + j, boundingBoxes.begin() + j+1);
                    }
                }
                topTwoBoundingBoxes.push_back(maxBoundingBox);
            }
            for(int i = 0; i < topTwoBoundingBoxes.size() && i < 2; i++) {
                rectangle(original.first, topTwoBoundingBoxes[i], Scalar(0, 0, 255), 1);
            }

            double x = 0;
            double y = 0;
            for(auto boundingBox : boundingBoxes) {
                x += boundingBox.x;
                y += boundingBox.y;
            }
            if(boundingBoxes.size() > 0) {
                x /= boundingBoxes.size();
                y /= boundingBoxes.size();
            }
            visionTable->PutNumber("piTime", original.second);
            visionTable->PutNumber("targetX", x);
            visionTable->PutNumber("targetY", visionResolution[0] - y);
#ifdef NOTRASPI
            imshow( "Display window", test);
            imshow( "Display window2", thresholdedImage);
            waitKey(-1);
            image++;
#endif

        //Camera streaming and switching
        if(visionTable->ContainsKey("camera")) {
            string cameraSelection = visionTable->GetString("camera", "NULL");
            if(cameraSelection == gearCameraName) {
                if(gearCamera.IsConnected()) {
                    mjpegServer.SetSource(gearCamera);
                } else {
                    updateNullCamera("Gear camera not connected!");
                    mjpegServer.SetSource(*nullSource);
                }
            } else if(cameraSelection == frontCameraName) {
                if(visionCamera.IsOpen()) {
                    Mat frame = original.first;
                    visionCameraSource->PutFrame(frame);
                    mjpegServer.SetSource(*visionCameraSource);
                } else {
                    updateNullCamera("Front camera not connected!");
                    mjpegServer.SetSource(*nullSource);
                }
            } else {
                cout << "Error: Invalid \"camera\" value found on Vision Network Table!" << endl;
                updateNullCamera("Invalid camera value on NT!");
                mjpegServer.SetSource(*nullSource);
            }
        } else {
            cout << "Network table not connected!" << endl;
            if(gearCamera.IsConnected()) {
                mjpegServer.SetSource(gearCamera);
            } else if(visionCamera.IsOpen()) {
                Mat frame = original.first;
                visionCameraSource->PutFrame(frame);
                mjpegServer.SetSource(*visionCameraSource);
            } else {
                updateNullCamera("NT not connected!");
                mjpegServer.SetSource(*nullSource);
            }
        }
    }
}