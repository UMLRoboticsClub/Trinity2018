#include "robot.h"
#include "drive.h"
#include "pins.h"
#include "motor.h"
#include "point.h"
#include "gpio.h"
#include "MPU6050.h"
#include "opticalflow.h"

#include <pigpiod_if2.h>
#include <csignal>
#include <chrono>
#include <iostream>
#include <cmath>
#include <iomanip>
using std::setw;

using std::cout;
using std::endl;

typedef std::chrono::high_resolution_clock Clock;
typedef std::chrono::time_point<std::chrono::high_resolution_clock> timePoint;

Motor Drive::motorA(motor1APin, motor1BPin);
Motor Drive::motorB(motor2APin, motor2BPin);
Motor Drive::motorC(motor3APin, motor3BPin);

MPU6050 Drive::mpu;
opticalFlow Drive::OF;

double timeDelta(timePoint end, timePoint begin){
    return std::chrono::duration_cast<std::chrono::microseconds>(end-begin).count()/1000000.0;
}

double Drive::getGyroData(){
    return mpu.getRotationZ()*500/32768;
}

auto now = Clock::now();
auto last = Clock::now();

double updateTime(){
    last = now;
    now = Clock::now();
    return timeDelta(now, last);
}

void normalize(double& a, double& b, double& c){
    double am = fabs(a), bm = fabs(b), cm = fabs(c);
    double largest = am > bm ? (am > cm ? am : cm) : (bm > cm ? bm : cm);
    if(largest > 255){
        a = a*255/largest;
        b = b*255/largest;
        c = c*255/largest;
    }
}

void Drive::drive(DoublePoint target) {
//    motorA.set(255);
//    time_sleep(1);
//    motorA.set(0);
//    motorB.set(255);
//    time_sleep(1);
//    motorB.set(0);
//    exit(1);
    double angle = atan2((target.y-robotPos.y), (target.x-robotPos.x));
//    double ASpeed = 255*cos(MVA_ANGLE - angle);
//    double BSpeed = 255*cos(MVB_ANGLE - angle);
//    double CSpeed = 255*cos(MVC_ANGLE - angle);
//        motorA.set(ASpeed);
//        motorB.set(BSpeed);
//        motorC.set(CSpeed);
//    time_sleep(1);
//        motorA.set(0);
//        motorB.set(0);
//        motorC.set(0);
//    exit(1);

        //int max_power=255;
    //auto start = Clock::now();
    DoublePoint error_prior;
    DoublePoint integral;
    DoublePoint derivative;
    DoublePoint output;
    double kp   = 40;
    double ki   = 0;
    double kd   = 0;
    double eps  = 1;
    double veps = 2;
    double vRad = 0;
    double theta = 0;
    //double gyroThreshold = 0.1;
    //double timeDelta = 0;
    //double bias;
    
    DoublePoint error = target - getRobotPos();
    DoublePoint vel(0, 0);

    while(error.magnitude() > eps || vel.magnitude() > veps){
        //updateTime();
        // auto deltaTime = std::chrono::duration_cast<std::chrono::microseconds>(currTime - prevTime).count()/1000000.0;
        double deltaTime = updateTime();
        robotPos += OF.readMotion();

        //I could accrue our displacement based on theta and correct at the end?
        //gyro correction happens here
        //so if gyro gives a velocity non zero, increase or decrease motor values respectively.
        vRad = Drive::getGyroData();
        theta += vRad*deltaTime;
        //  cout << theta << endl;
        //  accruedError.x += sin(theta*DEG_TO_RAD)*timeDelta;
        //  accruedError.y += (1 - cos(theta*DEG_TO_RAD))*timeDelta;
        error = target - robotPos;
        integral += error * deltaTime;
        derivative = (error - error_prior) / deltaTime;
        output =  error * kp + integral * ki + derivative * kd;
        error_prior = error;
        angle = atan2(output.y, output.x);
        cout << setw(10) << error.x << " " << setw(10) <<  error.y << " " << setw(10) << output.x << " " << setw(10) << output.y << " ";

        double ASpeed = output.magnitude() * cos(MVA_ANGLE - angle);
        double BSpeed = output.magnitude() * cos(MVB_ANGLE - angle);
        double CSpeed = output.magnitude() * cos(MVC_ANGLE - angle);
        normalize(ASpeed,BSpeed,CSpeed);
        motorA.set(ASpeed);
        motorB.set(BSpeed);
        motorC.set(CSpeed);
        cout << setw(10) << ASpeed << " " << setw(10) << BSpeed << " " << setw(10) << CSpeed << endl;
        //cout<< "Output = "<<output.x<<" "<<output.y<<endl;

        //cout << "ASpeed = " << ASpeed << endl;
        //cout << "BSpeed = " << BSpeed << endl;
        //cout << "CSpeed = " << CSpeed << endl;

        /* if(vRad > gyroThreshold || vRad < -gyroThreshold){//alternatively do it based on theta rather than vRad.  Give it a shot
           ASpeed += vRad*ki;
           BSpeed += vRad*ki; 
           CSpeed += vRad*ki;

           normalize(ASpeed, BSpeed, CSpeed);

           motorA.set(ASpeed);
           motorB.set(BSpeed);
           motorC.set(CSpeed);
           }*/
    }

    cout << "done" << endl;

    motorA.set(0);
    motorB.set(0);
    motorC.set(0);
}

void Drive::rotate(double error) {
    double error_prior = 0;
    double integral = 0;
    double derivative = 0;
    double output = 0;
    double bias = 0;
    double kp=5;
    double ki=.2;
    double kd=0;
    double eps=1;
    double veps=0.001;
    double vel=0;

    while(error > eps || vel > veps){
        cout << error << endl;
        double deltaTime = updateTime();
        vel = Drive::getGyroData();
        //cout << vel << endl;
        //double robotAngle = getRobotAngle();
        setRobotAngle(getRobotAngle()+vel*deltaTime);
        //setRobotAngle(robotAngle);

        error -= vel*deltaTime;
        integral = integral + (error * deltaTime);
        derivative = (error - error_prior) / deltaTime;
        output = kp * error + ki * integral + kd * derivative + bias;
        error_prior = error;
        if(output < -255) output = -255;
        if(output > 255) output = 255;
        //cout<<output<<endl;    
        motorA.set(-output);
        motorB.set(-output);
        motorC.set(-output);
    }
    motorA.set(0);
    motorB.set(0);
    motorC.set(0);
}



