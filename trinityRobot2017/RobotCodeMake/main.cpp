#include "robot.h"
#include "drive.h"
#include "pins.h"
#include "motor.h"
#include "point.h"
#include "gpio.h"
#include "MPU6050.h"

#include <pigpiod_if2.h>
#include <csignal>
#include <chrono>
#include <unistd.h>
#include <iostream>
#include <cmath>

//using namespace std;

typedef std::chrono::high_resolution_clock Clock;
typedef std::chrono::time_point<std::chrono::high_resolution_clock> timePoint;

Motor motorA(motor1APin, motor1BPin);
Motor motorB(motor2APin, motor2BPin);
Motor motorC(motor3APin, motor3BPin);
MPU6050 mpu;

double x=0;
double y=0;

double deltaTime(timePoint end, timePoint begin){
    return std::chrono::duration_cast<std::chrono::microseconds>(end-begin).count()/1000000.0;
}

auto now = Clock::now();
auto last = Clock::now();
double updateTime(){
    last = now;
    now = Clock::now();
    return deltaTime(now, last);
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

void drive(double x,double y){

    double angle = atan2(x, y);
    auto start = Clock::now();
    double ASpeed = 255*cos(MVA_ANGLE - angle);
    double BSpeed = 255*cos(MVB_ANGLE - angle);
    double CSpeed = 255*cos(MVC_ANGLE - angle);
    double vRad = 0;
    double theta = 0;
    double timeDelta = 0;
    double k = .01;
    double gyroThreshold = 5;
    double endTime = sqrt(x*x+y*y);
    motorA.set(ASpeed);
    motorB.set(BSpeed);
    motorC.set(CSpeed);
    DoublePoint accruedError(0, 0);
    while(deltaTime(Clock::now(), start) < endTime){
        //I could accrue our displacement based on theta and correct at the end?
        //gyro correction happens here
        //so if gyro gives a velocity non zero, increase or decrease motor values respectively.
        timeDelta = updateTime();
        vRad = mpu.getRotationZ()*250/32678;
        theta += vRad*timeDelta;
        //  cout << theta << endl;
        //  accruedError.x += sin(theta*DEG_TO_RAD)*timeDelta;
        //  accruedError.y += (1 - cos(theta*DEG_TO_RAD))*timeDelta;

        if(vRad > gyroThreshold || vRad < -gyroThreshold){//alternatively do it based on theta rather than vRad.  Give it a shot
            ASpeed += vRad*k;
            BSpeed += vRad*k; 
            CSpeed += vRad*k;

            normalize(ASpeed, BSpeed, CSpeed);

            motorA.set(ASpeed);
            motorB.set(BSpeed);
            motorC.set(CSpeed);
        }
    }
    motorA.set(0);
    motorB.set(0);
    motorC.set(0);
    //cout << accruedError.x << " " << accruedError.y << std::endl;
    //if(accruedError.x > 0.0001 || accruedError.x < -.0001 || accruedError.y > 0.0001 || accruedError.y < -0.0001)
    //    drive(accruedError.x, accruedError.y);
}

//void rotate(error){
//  motorA.set(255);
//  motorB.set(255);
//  motorC.set(255);
//  time_sleep(2);
//  motorA.set(0);
//  motorB.set(0);
//  motorC.set(0);
//}

void signalHandler(int signum){
    /*gpio_write(pi, motor1A, 0);
      gpio_write(pi, motor1B, 0);
      gpio_write(pi, motor2A, 0);
      gpio_write(pi, motor2B, 0);
      gpio_write(pi, motor3A, 0);
      gpio_write(pi, motor3B, 0);
      exit(1);*/
    throw 3;
}

int main() {
    GPIO gpio;
    Drive::mpu.initialize();

    signal(SIGINT , signalHandler);
    signal(SIGABRT, signalHandler);
    signal(SIGFPE , signalHandler);
    signal(SIGILL , signalHandler);
    signal(SIGSEGV, signalHandler);
    signal(SIGTERM, signalHandler);
    signal(SIGHUP , signalHandler);

    //Robot sam;
    //sam.start();
    //MazeMapper mapper;
    //mapper.testFindNextTarget();
    //mapper.testCreateTargetPath();
    //mapper.testOptimizePath();
    //mapper.testPathIsBlocked();
    //mapper.testIsDiag();
    //mapper.testComputePathLength();
    //mapper.testComputeDistanceField();

    //double a, b;

    //rotate();
    // while(1){
    // std::cin >> a >> b;
    // drive(a, b);
    //}
    Drive::rotate(90);
    //drive(1, 0);
    //drive(0, 1);
    //drive(-1, 0);
    //drive(0, -1);
    // rotate();
    return 0;
}


