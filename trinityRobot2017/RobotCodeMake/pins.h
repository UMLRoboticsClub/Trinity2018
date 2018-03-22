#pragma once

/*
Status led:   2 (could be changed)
Power button: 3 (could be changed)
Lidar serial: 15

I have some 8 and 16 IO expanders if 
we need more pins. Won't work with PWM
-Jackson
*/

#define motor1APin 1
#define motor1BPin 2
#define motor2APin 3
#define motor2BPin 4
#define motor3APin 5
#define motor3BPin 6

#define solenoidPin 7
#define lidarMotorPin 8

#define servo1APin 9
#define servo1BPin 10
#define servo2APin 11
#define servo2BPin 12

#define redLedPin 13
#define greenLedPin 14
#define blueLedPin 15

#define colorSensorPin 16
#define irSensorPin 17
#define imuSDAPin 18 //BCM 2!
#define imuSCLPin 19 //BCM 3!
