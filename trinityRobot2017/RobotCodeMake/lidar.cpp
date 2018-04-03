#include "lidar.h"

Lidar::Lidar(){
    //initLidar();
}

Lidar::~Lidar(){
    xv11lidar_close(lidar);
}

void Lidar::initLidar(){
    cout << "Starting lidar..." << endl;
    xv11lidar_close(lidar);
    lidar = xv11lidar_init(serialDevice, laserFramesPerRead, crcTolerancePercent);
    cout << "Lidar started" << endl;
}

void Lidar::handleBadInput(){
    if(!badData){
        cerr << "Error reading data..." << endl;
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
