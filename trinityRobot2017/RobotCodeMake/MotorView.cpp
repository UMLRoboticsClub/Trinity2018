/* 
 * File:   main.cpp
 * Author: Mary and Sana
 * Convert Cartesian coordinate into Motor view vectors
 */

#include <cstdlib>
#include <math.h>
#include "MotorView.h"

MotorView::MotorView(Point point, double deltaT) {
    this->point = point;
    this->deltaT = deltaT;
}


// convert from degree to radian

double toRadian(double alpha) {
    return alpha * DEG_TO_RAD;
}

// linear movement of robot 

void MotorView::move(double angle, double displacement) {
    double velocity = displacement / this->deltaT;
    setMotorVectorA(velocity * cos(MVA_ANGLE - angle));
    setMotorVectorB(velocity * cos(MVB_ANGLE - angle));
    setMotorVectorC(velocity * cos(MVC_ANGLE - angle));
}

// compute drive from Cartesian coordinate and change in time

void MotorView::computeDrive() {
    double thetaAngle = atan(point.getYcoord() / point.getXcoord());
    double angle = toRadian(thetaAngle); // convert angle from degree to radian
    double disp = sqrt(pow(point.getXcoord(), 2) + pow(point.getYcoord(), 2));

    move(angle, disp);
}

MotorView::~MotorView() {
}

void MotorView::setMotorVectorC(double motorVectorC) {
    this->motorVectorC = motorVectorC;
}

double MotorView::getMotorVectorC() const {
    return motorVectorC;
}

void MotorView::setMotorVectorB(double motorVectorB) {
    this->motorVectorB = motorVectorB;
}

double MotorView::getMotorVectorB() const {
    return motorVectorB;
}

void MotorView::setMotorVectorA(double motorVectorA) {
    this->motorVectorA = motorVectorA;
}

double MotorView::getMotorVectorA() const {
    return motorVectorA;
}

