#pragma once

#include "MPU6050.h"
#include "opticalflow.h"
#include "globals.h"
#include "constants.h"
#include "motor.h"

const double DEG_TO_RAD = M_PI / 180.0;
const double MVA_ANGLE = M_PI - (150 - 150)* DEG_TO_RAD;
const double MVB_ANGLE = M_PI - (30  - 150)* DEG_TO_RAD;
const double MVC_ANGLE = M_PI - (270 - 150)* DEG_TO_RAD;


class Drive {
    public:
        Drive();
        static MPU6050 mpu;
        static opticalFlow OF;
        void drive(DoublePoint target );
        void rotate(double radians); //contains a rotate PID loop
        //3 motors+1 accelerometer
        static double getGyroData();
//    private:
        Motor motorA, motorB, motorC;
};
