#pragma once

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

Mat original, BWImage, visionThresholdedImage;
vector<vector<Point>> contours;
vector<Rect> boundingBoxes;
vector<Vec4i> hierarchy;
int image = 1;


void process(pair<Mat, time_t> original, shared_ptr<NetworkTable> visionTable) {
#ifdef NOTRASPI
    visionCamera.UseFileInput(true);
            visionCamera.SetFileInputLocation("../images/practiceImages/" + to_string(image) + ".jpg", MULTI_IMAGE);
#endif
    cvtColor(original.first, BWImage, COLOR_RGB2GRAY);
    threshold(BWImage, visionThresholdedImage, 55, 255, ThresholdTypes::THRESH_BINARY);
    contours.clear();
    hierarchy.clear();
    findContours(visionThresholdedImage, contours, hierarchy, CV_RETR_TREE, CV_CHAIN_APPROX_SIMPLE, Point(0, 0) );
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
        circle(original.first, topTwoBoundingBoxes[i].tl(), 1, Scalar(255, 0, 0));
    }
    Point center;
    if(!boundingBoxes.empty()) {
        center.x = 0;
        center.y = 0;
        for(auto boundingBox : boundingBoxes) {
            center.x += boundingBox.x;
            center.y += boundingBox.y;
        }
        if(boundingBoxes.size() > 0) {
            center.x /= boundingBoxes.size();
            center.y /= boundingBoxes.size();
        }
    } else {
        center.x = -1;
        center.y = -1;
    }
    circle(original.first, center, 2, Scalar(0, 255, 0));
    visionTable->PutNumber("piTime", original.second);
    visionTable->PutNumber("targetX", center.x);
    visionTable->PutNumber("targetY", visionResolution[0] - center.y);
#ifdef NOTRASPI
    imshow( "Display window", original.first);
            imshow( "Display window2", thresholdedImage);
            waitKey(-1);
            image++;
#endif
}