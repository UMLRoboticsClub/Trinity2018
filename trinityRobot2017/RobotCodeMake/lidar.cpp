#include "lidar.h"
#include <vector>
#include <deque>
Lidar::Lidar(){
    cout << "here" << endl;
    initLidar();
}

Lidar::~Lidar(){
    xv11lidar_close(lidar);
}

void Lidar::initLidar(){
    cout << "Starting lidar..." << endl;
    xv11lidar_close(lidar);
    cout << "closed..." << endl;
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
    cout << index << " " << frame[index].speed << endl;
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

