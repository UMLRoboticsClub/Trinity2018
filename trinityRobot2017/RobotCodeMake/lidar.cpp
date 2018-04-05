#include "lidar.h"
#include "pins.h"
#include "logger.h"
#include <pigpiod_if2.h>

#define LIDAR_MOTOR_SPEED 100

Lidar::Lidar(){
    set_PWM_dutycycle(0, lidarMotorPin, LIDAR_MOTOR_SPEED);
    initLidar();
}

Lidar::~Lidar(){
    xv11lidar_close(lidar);
    set_PWM_dutycycle(0, lidarMotorPin, 0);
}

void Lidar::initLidar(){
    Logger::Log("Starting lidar...");
    xv11lidar_close(lidar);
    lidar = xv11lidar_init(serialDevice, laserFramesPerRead, crcTolerancePercent);
    Logger::Log("Lidar started");
}

void Lidar::handleBadInput(){
    if(!badData){
    Logger::Log("Error reading data...");
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
    return frame[index].speed / 64;
}

void Lidar::processFrame(){
    cout << getRPM(0) << endl;

}

void Lidar::scan(){
    if(!xv11lidar_read(lidar, frame)){
        processFrame();
    } else {
        handleBadInput();
    }
}
