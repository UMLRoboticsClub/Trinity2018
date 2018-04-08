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

#define MINIMUM 75

using std::setw;

using std::cout;
using std::endl;

typedef std::chrono::high_resolution_clock Clock;
typedef std::chrono::time_point<std::chrono::high_resolution_clock> timePoint;

MPU6050 Drive::mpu;
opticalFlow Drive::OF;

Drive::Drive():
    motorA(motor1APin, motor1BPin),
    motorB(motor2APin, motor2BPin),
    motorC(motor3APin, motor3BPin)
{}

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

void normalize(double& a, double& b, double& c, int max = 255){
    double am = fabs(a), bm = fabs(b), cm = fabs(c);
    double largest = am > bm ? (am > cm ? am : cm) : (bm > cm ? bm : cm);
    if(largest > max){
        a = a*max/largest;
        b = b*max/largest;
        c = c*max/largest;
    }
}
void normalizeUp(double& a, double& b, double& c, int max = 255){
    double am = fabs(a), bm = fabs(b), cm = fabs(c);
    double largest = am > bm ? (am > cm ? am : cm) : (bm > cm ? bm : cm);
    if(largest < max){
        a = a*max/largest;
        b = b*max/largest;
        c = c*max/largest;
    }
}

void Drive::drive(DoublePoint target) {
    
    
    //    motorA.set(255);
    //    motorB.set(255);
    //    motorC.set(255);
    //    time_sleep(4);
    //    exit(1);
    //    motorA.set(255);
    //    time_sleep(1);
    //    motorA.set(0);
    //    motorB.set(255);
    //    time_sleep(1);
    //    motorB.set(0);
    //    exit(1);
    double angle = atan2((target.y-getRobotPos().y), (target.x-getRobotPos().x));
    setDirection(angle);
    
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
    double kp   = 20;
    double ki   = 15;
    double kd   = 5;
    double gyroK = 5;
    double eps  = 2;
    double veps = 2;
    double vRad = 0;
    double theta = 0;
    double gyroThreshold = .5;
    double deltaTime = 0;
    double frozenCount = 0;
    //double bias;

    DoublePoint error = target - getRobotPos();
    DoublePoint vel(0, 0);

    while(error.magnitude() > eps || vel.magnitude() > veps){

        //updateTime();
        //auto deltaTime = updateTime();//std::chrono::duration_cast<std::chrono::microseconds>(currTime - prevTime).count()/1000000.0;
        deltaTime = updateTime();
       // DoublePoint oldPos(getRobotPos());
       // DoublePoint deltaPos = OF.readMotion();
       // oldPos.x += deltaPos.x * cos(getRobotAngle()) - deltaPos.y * sin(getRobotAngle());
       // oldPos.y += deltaPos.x * sin(getRobotAngle()) + deltaPos.y * cos(getRobotAngle());
       // setRobotPos(oldPos);
        //robotPos += OF.readMotion();

        //I could accrue our displacement based on theta and correct at the end?
        //gyro correction happens here
        //so if gyro gives a velocity non zero, increase or decrease motor values respectively.
        vRad = Drive::getGyroData();
        theta += vRad*deltaTime;
        setRobotAngle(getRobotAngle() + vRad*deltaTime*M_PI/180);
        //robotAngle += vRad*deltaTime;
        //  cout << theta << endl;
        //  accruedError.x += sin(theta*DEG_TO_RAD)*timeDelta;
        //  accruedError.y += (1 - cos(theta*DEG_TO_RAD))*timeDelta;
        error = target - getRobotPos();
        if((error.x > 0) != (error_prior.x < 0))
            integral.x = 0;
        if((error.y > 0) != (error_prior.y > 0))
            integral.y = 0;
        // motion profiling
        if(error.magnitude() <= 10){
            integral += error * deltaTime;
        }
        derivative = (error - error_prior) / deltaTime;
        if(derivative.magnitude() == 0)
            frozenCount += deltaTime;
        else
            frozenCount = 0;
        if(frozenCount > 1){
            motorA.set(0);
            motorB.set(0);
            motorC.set(0);
            cout << "robotPos in PID: " << getRobotPos() << endl;
            return;
        }
        output =  error * kp + integral * ki + derivative * kd;
        error_prior = error;
        angle = atan2(output.y, output.x);
        //cout << setw(10) << error.x << " " << setw(10) <<  error.y << " " << setw(10) << output.x << " " << setw(10) << output.y << " " << setw(10) << integral.x << " " << setw(10) << integral.y << " " << setw(10) << derivative.x << " " << setw(10) << derivative.y;

        double ASpeed = output.magnitude() * cos(MVA_ANGLE - angle);
        double BSpeed = output.magnitude() * cos(MVB_ANGLE - angle);
        double CSpeed = output.magnitude() * cos(MVC_ANGLE - angle);
        normalize(ASpeed,BSpeed,CSpeed);
        //cout << setw(10) << ASpeed << " " << setw(10) << BSpeed << " " << setw(10) << CSpeed << endl;
        //cout<< "Output = "<<output.x<<" "<<output.y<<endl;

        //cout << "ASpeed = " << ASpeed << endl;
        //cout << "BSpeed = " << BSpeed << endl;
        //cout << "CSpeed = " << CSpeed << endl;

        if(theta > gyroThreshold || theta < -gyroThreshold){//alternatively do it based on theta rather than vRad.  Give it a shot

            ASpeed -= vRad*gyroK;
            BSpeed -= vRad*gyroK; 
            CSpeed -= vRad*gyroK;

        }
        normalizeUp(ASpeed, BSpeed, CSpeed, 75);
        normalize(ASpeed, BSpeed, CSpeed, 150);


        motorA.set(ASpeed);
        motorB.set(BSpeed);
        motorC.set(CSpeed);
        // cout << robotPos.x << " " << robotPos.y << endl;
    }
    //cout << robotPos.x << " " << robotPos.y;

    //continue gathering data to snure accurate robotPos and robotAngle
    //start = Clock::now();
    //while(timeDelta(now, start) < 0.5){
    //    double deltaTime = updateTime();
    //    setRobotPos(getRobotPos()+OF.readMotion());
    //    //robotPos += OF.readMotion();
    //    vRad = Drive::getGyroData();
    //    theta += vRad*deltaTime;
    //    setRobotAngle(getRobotAngle() + vRad*deltaTime);
    //    //robotAngle += vRad*deltaTime;
    //}
    setDirection(7);//goin nowhere, 7 is a random number :( 
    rotate(-theta);
    cout << "done" << endl;
    cout << "robotPos in PID: " << getRobotPos() << endl;
    //cout << robotPos.x << " " << robotPos.y << " " << robotAngle << endl;
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
    double kp=2;
    double ki=3;
    double kd=0.1;
    double eps=5;
    double veps=0.01;
    double vel=0;

    while(abs(error) > eps || abs(vel) > veps){
        //cout << error << endl;
        double deltaTime = updateTime();
        vel = Drive::getGyroData();
        //cout << vel << endl;
        //double robotAngle = getRobotAngle();
        setRobotAngle(getRobotAngle()+vel*deltaTime*M_PI/180);
        //setRobotAngle(robotAngle);

        error -= vel*deltaTime;
        integral = integral + (error * deltaTime);
        derivative = (error - error_prior) / deltaTime;

        output = kp * error + ki * integral + kd * derivative + bias;

        error_prior = error;
        if(output < -255) output = -255;
        if(output > 255) output = 255;
        if(output > 0 && output < 75)
            output = 75;
        if(output < 0 && output > -75)
            output = -75;
        //cout<<output<<endl;    
        motorA.set(output);
        motorB.set(output);
        motorC.set(output);
    }
    motorA.set(0);
    motorB.set(0);
    motorC.set(0);
}



