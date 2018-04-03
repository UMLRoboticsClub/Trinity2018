#include "drive.h"
#include "pins.h"
#include "motor.h"
#include "point.h"

#include <chrono>
#include <cmath>
#include <pigpiod_if2.h>
#include <iostream>

typedef std::chrono::high_resolution_clock Clock;

auto startTime = Clock::now();
auto currTime = Clock::now();
auto prevTime = Clock::now();
double deltaTime;

void Drive::updateTime(){
    prevTime=currTime;
    currTime=Clock::now();
}
//angleDelta += updateAngle(std::chrono::duration_cast<std::chrono::microseconds>(currTime - prevTime).count());
Motor Drive::motorA(motor1APin, motor1BPin);
Motor Drive::motorB(motor2APin, motor2BPin);
Motor Drive::motorC(motor3APin, motor3BPin);
MPU6050 Drive::mpu;

//GPIO gpio;
//mpu.initialise();

void Drive::drive(DoublePoint target) {
    int max_power=255;
    double angle = atan((target.y-robotPos.y)/(target.x-robotPos.x));
    double disp = sqrt((target.x-robotPos.x)*(target.x-robotPos.x) + (target.y-robotPos.y)*(target.y-robotPos.y));
    DoublePoint error_prior;
    DoublePoint integral;
    DoublePoint derivative;
    DoublePoint output;
    double kp=0.1;
    double ki=0.1;
    double kd=0.1;
    double eps=0;
    double veps=0;
    DoublePoint error;
    DoublePoint vel;

    double bias;

    while(error.magnitude()>eps || vel.magnitude()>veps){
        updateTime();
        auto deltaTime = std::chrono::duration_cast<std::chrono::microseconds>(currTime - prevTime).count()/1000000.0;
        vel.x += mpu.getAccelerationX()*100*deltaTime;
        vel.y += mpu.getAccelerationY()*100*deltaTime;
        robotPos.x += vel.x*deltaTime;
        robotPos.y += vel.y*deltaTime;

        error = target - robotPos;
        integral = integral + (error * deltaTime);
        derivative = (error - error_prior) / deltaTime;
        output =  error * kp + integral * ki + derivative*kd+bias;
        error_prior = error;
        angle = atan(output.y/output.x);
        motorA.set(output.magnitude() * cos(MVA_ANGLE - angle));
        motorB.set(output.magnitude() * cos(MVB_ANGLE - angle));
        motorC.set(output.magnitude() * cos(MVC_ANGLE - angle));
    }

}

void Drive::rotate(double error) {
    double error_prior = 0;
    double integral = 0;
    double derivative = 0;
    double output = 0;
    double bias = 0;
    double kp=20;
    double ki=0;
    double kd=0;
    double eps=0.1;
    double veps=0.1;
    double vel=0;

    while(error > eps || vel > veps){
        updateTime();
        deltaTime = std::chrono::duration_cast<std::chrono::microseconds>(currTime - prevTime).count()/1000000.0;
        vel = mpu.getRotationZ()*deltaTime;
        robotAngle += vel*deltaTime;

        error -= vel*deltaTime;
        integral = integral + (error * deltaTime);
        derivative = (error - error_prior) / deltaTime;
        output = kp * error + ki * integral + kd * derivative + bias;
        error_prior = error;
        std::cout<<output<<endl;    
        motorA.set(-output);
        motorB.set(-output);
        motorC.set(-output);
    }
}
