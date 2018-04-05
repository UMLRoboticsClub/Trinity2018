#pragma once

#include "point.h"
#include <mutex>
//Robot Position -- set by Drive only
extern DoublePoint robotPos;
extern double robotAngle;
extern std::mutex occGridMutex;
extern std::mutex robotPosMutex;
extern std::mutex targetPointsMutex;
extern std::mutex robotAngleMutex;

extern DoublePoint getRobotPos();
extern void setRobotPos(DoublePoint newPos);

extern double getRobotAngle();
extern void setRobotAngle(double newangle);
//dem mutexes
