#include "drive.h"
#include "pins.h"
#include "motor.h"
#include "point.h"
#include "gpio.h"
#include "lidar.h"
#include "MPU6050.h"

#include <stdexcept>
#include <chrono>
#include <unistd.h>
#include <iostream>
#include <cmath>

#include "robot.h"
#include "logger.h"

#include <pigpiod_if2.h>
#include <csignal>

using std::cout;
using std::endl;

//#define testIR
//#define testMotors
//#define testSolenoid
//#define testGyro
#define testLidar
//#define testLED




int main() {
GPIO gpio;


set_mode(0, motor1APin, PI_OUTPUT);
set_mode(0, motor1BPin, PI_OUTPUT);
set_mode(0, motor2APin, PI_OUTPUT);
set_mode(0, motor2BPin, PI_OUTPUT);
set_mode(0, motor3APin, PI_OUTPUT);
set_mode(0, motor3BPin, PI_OUTPUT);
set_mode(0, solenoidPin, PI_OUTPUT);
set_mode(0, lidarMotorPin, PI_OUTPUT);
set_mode(0, redLedPin, PI_OUTPUT);
set_mode(0, greenLedPin, PI_OUTPUT);
set_mode(0, blueLedPin, PI_OUTPUT); //I think this is wrong
set_mode(0, irSensorPin, PI_INPUT);

#ifdef testIR
    while(1){
        cout << IRSensor::flameVisible() << endl;
        time_sleep(.1);
    }
#endif
#ifdef testMotors
    Motor motor(motor1APin, motor1BPin);
    Motor motor1(motor2APin, motor2BPin);
    Motor motor2(motor3APin, motor3BPin);
    //int power = 70;

    //motor.set(255);
    //motor1.set(255);
    //motor2.set(255);
    //while(1){}
    int power = 255;
    while(1){
        //set_PWM_dutycycle(0, motor1APin, 0);
        //set_PWM_dutycycle(0, motor2APin, 0);
        //set_PWM_dutycycle(0, motor3APin, 0);
        //set_PWM_dutycycle(0, motor1BPin, 0);
        //set_PWM_dutycycle(0, motor2BPin, 0);
        //set_PWM_dutycycle(0, motor3BPin, 0);
        //gpio_write(0, motor1APin, 0);
        //gpio_write(0, motor2APin, 0);
        //gpio_write(0, motor3APin, 0);
        //gpio_write(0, motor1BPin, 0);
        //gpio_write(0, motor2BPin, 0);
        //gpio_write(0, motor3BPin, 0);
        //

        motor.set(-power);
        motor1.set(-power);
        motor2.set(-power);
        time_sleep(3);

        motor.set(0);
        motor1.set(0);
        motor2.set(0);
        time_sleep(1);

        motor.set(power);
        motor1.set(power);
        motor2.set(power);
        time_sleep(3);

        motor.set(0);
        motor1.set(0);
        motor2.set(0);
        time_sleep(1);
    }
    //copy motortest
#endif
#ifdef testSolenoid
    while(1){
        gpio_write(0, solenoidPin, 1);
        time_sleep(1);
        gpio_write(0, solenoidPin, 0);
        time_sleep(1);
    }
#endif
#ifdef testGyro
    MPU6050 mpu;
    mpu.initialize();
    while(1){
        cout << mpu.getRotationZ() << endl;
    }
#endif
#ifdef testLidar
    Lidar lidar;

    while(1){
        lidar.scan();
    }
    set_PWM_dutycycle(0, 16, 0);
    pigpio_stop(0);
    //lidarTest
#endif
#ifdef testLED
    while(1){
        gpio_write(0, redLedPin, 1);
        gpio_write(0, greenLedPin, 0);
        gpio_write(0, blueLedPin, 0);
        time_sleep(0.25);
        gpio_write(0, redLedPin, 0);
        gpio_write(0, greenLedPin, 1);
        gpio_write(0, blueLedPin, 0);
        time_sleep(0.25);
        gpio_write(0, redLedPin, 0);
        gpio_write(0, greenLedPin, 0);
        gpio_write(0, blueLedPin, 1);
        time_sleep(0.25);
    }
    //ledTest
#endif
}
