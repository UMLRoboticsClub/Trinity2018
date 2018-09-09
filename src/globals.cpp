#include "globals.h"
#include "constants.h"

DoublePoint robotPos(GRID_SIZE_CM/2, GRID_SIZE_CM/2);

double robotAngle = 0;

double direction = 10;

std::mutex occGridMutex;
std::mutex robotPosMutex;
std::mutex targetPointsMutex;
std::mutex robotAngleMutex;
std::mutex directionMutex;

DoublePoint getRobotPos(){
    std::lock_guard<std::mutex> lock(robotPosMutex);
    return robotPos;
}

void setRobotPos(DoublePoint newPos){
    std::lock_guard<std::mutex> lock(robotPosMutex);
    robotPos = newPos;
}

double getRobotAngle(){
    std::lock_guard<std::mutex> lock(robotAngleMutex);
    return robotAngle;
}

void setRobotAngle(double newAngle){
    std::lock_guard<std::mutex> lock(robotAngleMutex);
    robotAngle = newAngle;
}

double getDirection(){
    std::lock_guard<std::mutex> lock(directionMutex);
    return direction;
}

void setDirection(double newDirect){
    std::lock_guard<std::mutex> lock(directionMutex);
    direction = newDirect;
}
