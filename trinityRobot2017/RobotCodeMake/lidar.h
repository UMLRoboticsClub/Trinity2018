#pragma once

#include "xv11lidar.h"
#include <iostream>
#include <chrono>

using std::cout;
using std::cerr;
using std::endl;
using namespace std::chrono;

class Lidar {
    public:
        Lidar();
        ~Lidar();
        void scan();

    private:
        const char* serialDevice = "/dev/ttyAMA0";
        const int serialBaudRate = 115200;
        //constexpr static const int laserFramesPerRead = 90; //each frame has 4 degrees
        constexpr static const int laserFramesPerRead = 1; //each frame has 4 degrees
        const int crcTolerancePercent = 5; //accept up to this many crc_failures each revolution [0, 100]
        const float restartTime = 0.1f;
        xv11lidar *lidar = NULL;
        xv11lidar_frame frame[laserFramesPerRead]; 
        bool badData = false;

        high_resolution_clock::time_point errorStartTime;

        void initLidar();
        void handleBadInput();
        int getRPM(int index);
        void processFrame();
};
