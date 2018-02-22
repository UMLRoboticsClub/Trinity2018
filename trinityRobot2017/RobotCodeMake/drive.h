#pragma once

#include "motor.h"

class Drive {
    public:
        Drive();
        void drive(int deltaX, int deltaY);
        void rotate(double radians); //contains a rotate PID loop
        //3 motors+1 accelerometer
    private:
        Motor motorA, motorB, motorC;
};
