#include "robot.h"
#include "drive.h"
#include "pins.h"
#include "motor.h"
#include "point.h"
#include "gpio.h"
#include "lidar.h"
#include "MPU6050.h"
#include <stdexcept>

#include <pigpiod_if2.h>
#include <csignal>
#include <chrono>
#include <unistd.h>
#include <iostream>
#include <cmath>

bool running = true;

void signalHandler(int signum){
//      gpio_write(0,Drive::motorA.pinA, 0);
//      gpio_write(0,Drive::motorA.pinB, 0);
//      gpio_write(0,Drive::motorB.pinA, 0);
//      gpio_write(0,Drive::motorB.pinB, 0);
//      gpio_write(0,Drive::motorC.pinA, 0);
//      gpio_write(0,Drive::motorC.pinB, 0);
//      exit(1);
    running = false;
//    //throw std::runtime_error("Exit signal caught, aborting...");
//
}

int main() {
    GPIO gpio;
   //Lidar l;

   //while(running){
   //    deque<int> scan = l.scan();
   //    for(int i = 0; i < scan.size(); i++)
   //        cout << scan[i] << ", ";
   //    cout << endl;
   //}
    Robot r;

    r.start();
    //Drive::mpu.initialize();
    //Drive::OF.init();
//    signal(SIGINT , signalHandler);
//    signal(SIGABRT, signalHandler);
//    signal(SIGFPE , signalHandler);
//    signal(SIGILL , signalHandler);
//    signal(SIGSEGV, signalHandler);
//    signal(SIGTERM, signalHandler);
//    signal(SIGHUP , signalHandler);

    //rotate();
    // while(1){
    // std::cin >> a >> b;
    // drive(a, b);
    //}
    //Drive::drive(DoublePoint(1,0));
   // Drive::rotate(60);
    //drive(1, 0);
    //drive(0, 1);
    //drive(-1, 0);
    //drive(0, -1);
    // rotate();
    return 0;
}

