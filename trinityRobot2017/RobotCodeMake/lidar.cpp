#include "lidar.h"
#include "pins.h"
#include "logger.h"
#include <pigpiod_if2.h>
#include <vector>
#include <deque>

#define LIDAR_MOTOR_SPEED 100

Lidar::Lidar(){
    set_mode(0, lidarMotorPin, PI_OUTPUT);
    set_PWM_dutycycle(0, lidarMotorPin, LIDAR_MOTOR_SPEED);
    initLidar();
}

Lidar::~Lidar(){
    Logger::log("Stopping lidar...");
    set_PWM_dutycycle(0, lidarMotorPin, 0);
    xv11lidar_close(lidar);
    Logger::log("Done stopping lidar");
}

void Lidar::initLidar(){
    Logger::log("Starting lidar...");
    xv11lidar_close(lidar);
    time_sleep(1);
    lidar = xv11lidar_init(serialDevice, laserFramesPerRead, crcTolerancePercent);
    Logger::log("Lidar started");
}

void Lidar::handleBadInput(){
    if(!badData){
    Logger::log("Error reading data...");
        errorStartTime = high_resolution_clock::now();
        badData = true;
    } else {
        duration<float, std::milli> ms = high_resolution_clock::now() - errorStartTime;
        if(ms.count() > restartTime){
            badData = false;
            initLidar();
        }
    }
}

int Lidar::getRPM(int index){
    //cout << index << " " << frame[index].speed << endl;
    return frame[index].speed / 64;
}

std::deque<int> Lidar::processFrame(){
    std::deque<int> scan(360);

    for(int i = 0; i < scan.size() / 4; i++){
        for(int j = 0; j < 4; j++){
            scan[(frame[i].index-0xA0)*4+j] = frame[i].readings[j].distance;
        }
    }

    return scan;
}

std::deque<int> Lidar::scan(){
    if(!xv11lidar_read(lidar, frame)){
        return processFrame();
    } else {
        handleBadInput();
        std::deque<int> empty;
        return empty;
    }
}
