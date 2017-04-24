#pragma once

#include "opencv2/imgproc.hpp"
#include "opencv2/highgui.hpp"
#include "COREVisionLib.h"
#include <string>

using namespace cv;
using namespace CORE;
using namespace std;

class captureImages {
public:
    captureImages(string fileSuffix, double minimumIntervalSeconds) {
        m_minimumIntervalSeconds = minimumIntervalSeconds;
        m_fileName = "/home/pi/capturedImages/";
        m_fileName += fileSuffix;
        time_t currentTime = time(0);
        struct tm* now = localtime(&currentTime);
        m_fileName += " " + to_string(now->tm_mon) + "-" + to_string(now->tm_mday) + "--" + to_string(now->tm_hour)
                    + "-" + to_string(now->tm_min) + " ";
        timer.Reset();
        timer.Start();
    }
    void recordImage(Mat image) {
        if(timer.Get() > m_minimumIntervalSeconds) {
            imwrite(m_fileName + to_string(m_fileNumber) + ".jpg", image);
            m_fileNumber++;
            timer.Reset();
            timer.Start();
        }
    }
private:
    CORETimer timer;
    string m_fileName;
    double m_minimumIntervalSeconds;
    int m_fileNumber = 1;
};
