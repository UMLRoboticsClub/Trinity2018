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
#include <unistd.h>
#include <iostream>
#include <cmath>

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
    //int max_power=255;
    auto start = Clock::now();
    double angle = atan((target.y-robotPos.y)/(target.x-robotPos.x));
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

    double vRad = 0;
    double theta = 0;
    double gyroThreshold = 0.1;
    double timeDelta = 0;
    double bias;

    while(error.magnitude()>eps || vel.magnitude()>veps){
        //updateTime();
       // auto deltaTime = std::chrono::duration_cast<std::chrono::microseconds>(currTime - prevTime).count()/1000000.0;
        double deltaTime = updateTime();
        robotPos += OF.readMotion();

        //I could accrue our displacement based on theta and correct at the end?
        //gyro correction happens here
        //so if gyro gives a velocity non zero, increase or decrease motor values respectively.
        vRad = mpu.getRotationZ()*250/32678;
        theta += vRad*deltaTime;
        //  cout << theta << endl;
        //  accruedError.x += sin(theta*DEG_TO_RAD)*timeDelta;
        //  accruedError.y += (1 - cos(theta*DEG_TO_RAD))*timeDelta;
        error = target - robotPos;
        integral = integral + (error * deltaTime);
        derivative = (error - error_prior) / deltaTime;
        output =  error * kp + integral * ki + derivative*kd+bias;
        error_prior = error;
        angle = atan(output.y/output.x);

        double ASpeed = output.magnitude()*cos(MVA_ANGLE - angle);
        double BSpeed = output.magnitude()*cos(MVB_ANGLE - angle);
        double CSpeed = output.magnitude()*cos(MVC_ANGLE - angle);
        motorA.set(ASpeed);
        motorB.set(BSpeed);
        motorC.set(CSpeed);

        if(vRad > gyroThreshold || vRad < -gyroThreshold){//alternatively do it based on theta rather than vRad.  Give it a shot
            ASpeed += vRad*ki;
            BSpeed += vRad*ki; 
            CSpeed += vRad*ki;

            normalize(ASpeed, BSpeed, CSpeed);

            motorA.set(ASpeed);
            motorB.set(BSpeed);
            motorC.set(CSpeed);
    }
    motorA.set(0);
    motorB.set(0);
    motorC.set(0);
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
        double deltaTime = updateTime();
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
