#include "drive.h"

Drive::Drive():
motorA(motor1APin, motor1BPin),
motorB(motor2APin, motor2BPin),
motorC(motor3APin, motor3BPin)
{}

void Drive::drive(int deltaX, int deltaY) {

}

void Drive::rotate(double radians) { //contains a rotate PID loop

}
