#include "globals.h"
#include "constants.h"

DoublePoint robotPos(GRID_SIZE_CM/2, GRID_SIZE_CM/2);

double robotAngle = 0;

std::mutex occGridMutex;
std::mutex robotPosMutex;
std::mutex targetPointsMutex;

DoublePoint getRobotPos(){
    std::lock_guard<std::mutex> lock(robotPosMutex);
    return robotPos;
}

void setRobotPos(DoublePoint newPos){
    std::lock_guard<std::mutex> lock(robotPosMutex);
    robotPos = newPos;
}
