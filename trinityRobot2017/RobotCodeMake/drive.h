#pragma once
#include "MPU6050.h"
#include "opticalflow.h"
#include "globals.h"
#include "constants.h"
#include "motor.h"
const double DEG_TO_RAD = M_PI / 180.0;
const double MVA_ANGLE = 150 * DEG_TO_RAD;
const double MVB_ANGLE = 30 * DEG_TO_RAD;
const double MVC_ANGLE = 270 * DEG_TO_RAD;

class Drive {
    public:
        static MPU6050 mpu;
        static opticalFlow OF;
        static void drive(DoublePoint target );
        static void rotate(double radians); //contains a rotate PID loop
        //3 motors+1 accelerometer
        static double getGyroData();
//    private:
        static Motor motorA, motorB, motorC;
};
