#pragma once

#include "point.h"
#include <mutex>
//Robot Position -- set by Drive only
extern DoublePoint robotPos;
extern double robotAngle;
extern double direction;
extern std::mutex occGridMutex;
extern std::mutex robotPosMutex;
extern std::mutex targetPointsMutex;
extern std::mutex robotAngleMutex;
extern std::mutex directionMutex;

extern DoublePoint getRobotPos();
extern void setRobotPos(DoublePoint newPos);

extern double getRobotAngle();
extern void setRobotAngle(double newangle);

extern double getDirection();
extern void setDirection(double newDirect);
//dem mutexes
