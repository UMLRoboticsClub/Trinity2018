#include "drive.h"
#include <cmath>
#include "pins.h"
#include <chrono>
#include "motor.h"
using namespace std;
typedef std::chrono::high_resolution_clock Clock;

startTime = Clock::now();
currTime = Clock::now();
prevTime = Clock::now();

void Drive::updateTime(){
    prevTime=currTime;
    currTime=Clock::now();
}
//angleDelta += updateAngle(std::chrono::duration_cast<std::chrono::microseconds>(currTime - prevTime).count());
Motor Drive::motorA(motor1APin, motor1BPin);
Motor Drive::motorB(motor2APin, motor2BPin);
Motor Drive::motorC(motor3APin, motor3BPin);

void Drive::drive(DoublePoint target) {
    int max_power=255;
    double angle = atan((target.y-robotPos.y)/(target.x-robotPos.x));
    double disp = sqrt((target.x-robotPos.x)*(target.x-robotPos.x) + (target.y-robotPos.y)*(target.y-robotPos.y));
    DoublePoint error_prior = 0;
    double integral = 0;
    double kp=0.1;
    double ki=0.1;
    double kd=0.1;
    double eps=0;
    double veps=0;
    DoublePoint error=0;
    DoublePoint vel=0;

    while(error>eps || vel>veps){
        updateTime();
        deltaTime = std::chrono::duration_cast<std::chrono::microseconds>(currTime - prevTime).count()/1000000;
        vel.x += gyroAcc.getAccelX()*deltaTime;
        vel.y += gyroAcc.getAccelY()*deltaTime;
        robotPos.x += vel.x*deltaTime;
        robotPos.y += vel.y*deltaTime;
        
        error = target - robotPos;
        integral = integral + (error * deltaTime);
        derivative = (error - error_prior) / deltaTime;
        output = kp * error + ki * integral + kd * derivative + bias;
        error_prior = error;
    
        motorA.set(output * cos(MVA_ANGLE - angle));
        motorB.set(output * cos(MVB_ANGLE - angle));
        motorC.set(output * cos(MVC_ANGLE - angle));
    }

}

void Drive::rotate(double error) {

    double error_prior = 0;
    double integral = 0;
    double kp=0.1;
    double ki=0.1;
    double kd=0.1;
    double eps=0;
    double veps=0;
    double vel=0;


    while(error>eps || vel>veps){
        updateTime();
        deltaTime = std::chrono::duration_cast<std::chrono::microseconds>(currTime - prevTime).count()/1000000;
        vel = gyroAcc.getGyroZ()*deltaTime;
        robotAngle += vel*deltaTime;
        
        error -= vel*deltaTime;
        integral = integral + (error * deltaTime);
        derivative = (error - error_prior) / deltaTime;
        output = kp * error + ki * integral + kd * derivative + bias;
        error_prior = error;
    
        motorA.set(output);
        motorB.set(output);
        motorC.set(output);
    }
}


