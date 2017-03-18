#include "cscore.h"
#include "networktables/NetworkTable.h"
#include "opencv2/core/core.hpp"
#include "COREVisionLib.h"
#include "llvm/StringRef.h"

#include <iostream>
#include <unistd.h>

using namespace std;
using namespace CORE;
using namespace cs;
using namespace llvm;

const StringRef ipAddress = "10.20.62.63";
const StringRef gearCameraName = "gear";
const StringRef frontCameraName = "front";

int main() {
    CORETimer timeout;
    shared_ptr<NetworkTable> visionTable;
    UsbCamera gearCamera("Gear Camera", 0);
    gearCamera.SetVideoMode(VideoMode::kMJPEG, 416, 240, 30);
    UsbCamera frontCamera("Front Camera", 1);
    frontCamera.SetVideoMode(VideoMode::kMJPEG, 416, 240, 30);

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

    //Front Camera
    timeout.Reset();
    timeout.Start();
    while(!frontCamera.IsConnected()) {
        cout << "Waiting for front camera" << endl;
        usleep(100000); //0.1 seconds
        if(timeout.Get() > 1) {
            cout << "Error! Could not initialize front camera!" << endl;
            break;
        }
    }

    NetworkTable::SetClientMode();
    NetworkTable::SetIPAddress(ipAddress);
    NetworkTable::Initialize();
    visionTable = NetworkTable::GetTable("Vision");

    MjpegServer mjpegServer("httpserver", 5800);
    mjpegServer.SetSource(gearCamera);

    while(true) {
        string cameraSelection = visionTable->GetString("camera", "NULL");
        if(cameraSelection == gearCameraName) {
            mjpegServer.SetSource(gearCamera);
        } else if (cameraSelection == frontCameraName) {
            mjpegServer.SetSource(frontCamera);
        } else if (cameraSelection == "NULL") {
            cout << "Error: \"camera\" value not found on Vision Network Table!" << endl;
        } else {
            cout << "Error: Invalid \"camera\" value found on Vision Network Table!" << endl;
        }
        //usleep(100000); //0.1 seconds
    }
}