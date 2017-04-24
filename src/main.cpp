#include "cscore.h"
#include "networktables/NetworkTable.h"
#include "opencv2/imgproc.hpp"
#include "opencv2/highgui.hpp"
#include "COREVisionLib.h"
#include "llvm/StringRef.h"

#include <iostream>
#include <unistd.h>
#include <chrono>
#include "visionProcessing.h"
//#include "captureImages.h"

//#define MANUALIMAGEMODE

using namespace std;
using namespace CORE;
using namespace cs;
using namespace cv;
using namespace llvm;

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
//captureImages pegCapture("peg", 0.5);

shared_ptr<NetworkTable> visionTable;

int main() {
    NetworkTable::SetClientMode();
    NetworkTable::SetTeam(2062);
    NetworkTable::Initialize();
    visionTable = NetworkTable::GetTable("Vision");
    visionTable->SetUpdateRate(0.01);

    gearCamera.SetVideoMode(VideoMode::kMJPEG, streamResolution[0], streamResolution[1], 30);
    backCamera.SetVideoMode(VideoMode::kMJPEG, streamResolution[0], streamResolution[1], 30);
    visionCamera.SetVideoMode(VideoMode::kMJPEG, visionResolution[0], visionResolution[1], 30);

    gearCameraSink = new CvSink("Gear Camera Processing");
    backCameraSink = new CvSink("Back Camera Processing");
    visionCameraSink = new CvSink("Vision Camera Processing");

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

    double visionCameraTimestamp, gearCameraTimestamp;

    Mat originalBackCameraFrame, BWBackFrame;

    bool doingVision;

    while(true) {
        doingVision = false;
        if(visionTable->ContainsKey("camera")) {
            selectedCamera = visionTable->GetString("camera", "gear");
        } else {
            selectedCamera = "gear";
        }

        //Peg Vision
        if(visionCamera.IsConnected()) {
            if(visionTable->ContainsKey("disablePegVision") && visionTable->GetBoolean("disablePegVision", false)) {

            } else {
                visionCameraTimestamp = visionCameraSink->GrabFrame(originalVisionCameraFrame);
#ifdef MANUALIMAGEMODE
                originalVisionCameraFrame = imread("../pegImages/practiceImages/" + to_string((int)visionImage)
                                                   + "_640x360.jpg", IMREAD_COLOR);
                visionImage += 0.1;
#endif
                if(originalVisionCameraFrame.empty()) {
                    cout << "Error in getting vision frame!" << endl;
                } else {
                    //pegCapture.recordImage(originalVisionCameraFrame);
                    processVisionCameraFrame(originalVisionCameraFrame, visionCameraTimestamp, visionTable);
                    doingVision = true;
                }
            }
        }

        //Gear Vision
        if(gearCamera.IsConnected()) {
            if(visionTable->ContainsKey("disableGearVision") && visionTable->GetBoolean("disableGearVision", false)) {
                if(selectedCamera == gearCameraName) {
                    gearCameraSink->GrabFrame(originalGearCameraFrame);
                    flip(originalGearCameraFrame, originalGearCameraFrame, -1);
                }
            } else {
                gearCameraTimestamp = gearCameraSink->GrabFrame(originalGearCameraFrame);
                flip(originalGearCameraFrame, originalGearCameraFrame, -1);
#ifdef MANUALIMAGEMODE
                /*originalGearCameraFrame = imread("../gearImages/" + to_string((int)gearImage) + "_640x360.jpg",
                         IMREAD_COLOR);
                gearImage += 0.1;*/
#endif
                if(originalGearCameraFrame.empty()) {
                    cout << "Error in getting gear frame!" << endl;
                } else {
                    processGearCameraFrame(originalGearCameraFrame, gearCameraTimestamp, visionTable);
                    doingVision = true;
                }
            }
        }

        if(!doingVision) {
            if(selectedCamera == gearCameraName) {
                if(gearCamera.IsConnected() && !originalGearCameraFrame.empty()) {
                    cvtColor(originalGearCameraFrame, BWGearFrame, COLOR_RGB2GRAY);
                    gearStreamSource.PutFrame(BWGearFrame);
                    driverVisionServer.SetSource(gearStreamSource);
                }
            } else {
                if(backCamera.IsConnected()) {
                    backCameraSink->GrabFrame(originalBackCameraFrame);
                    cvtColor(originalBackCameraFrame, BWBackFrame, COLOR_RGB2GRAY);
                    backStreamSource.PutFrame(BWBackFrame);
                    driverVisionServer.SetSource(backStreamSource);
                }
            }
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
}