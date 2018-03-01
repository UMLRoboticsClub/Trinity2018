/* 
 * File:   main.cpp
 * Author: Mary and Sana
 * Convert Cartesian coordinate into Motor view vectors
 */

#include <string>
#include <math.h>
#include "Point.h"

#ifndef MOTORVIEW_H
#define MOTORVIEW_H

const double DEG_TO_RAD = M_PI / 180.0;
const double MVA_ANGLE = 150 * DEG_TO_RAD;
const double MVB_ANGLE = 30 * DEG_TO_RAD;
const double MVC_ANGLE = 270 * DEG_TO_RAD;


class MotorView {
public:
    MotorView(Point point, double deltaT);
    void move(double angle, double displacement);
    void computeDrive();
    virtual ~MotorView();
    void setMotorVectorC(double motorVectorC);
    double getMotorVectorC() const;
    void setMotorVectorB(double motorVectorB);
    double getMotorVectorB() const;
    void setMotorVectorA(double motorVectorA);
    double getMotorVectorA() const;
private:
    double deltaT;
    Point point;
    double motorVectorA;
    double motorVectorB;
    double motorVectorC;
};

#endif /* MOTORVIEW_H */

