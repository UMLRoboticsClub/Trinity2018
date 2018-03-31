#ifndef ROBOT_H
#define ROBOT_H

#include "point.h"
#include "mazemapper.h"
#include "gamestate.h"
#include "globals.h"
#include "constants.h"
#include "drive.h"
#include "colorsensor.h"
#include "irsensor.h"
//#include "camera.h"
#include "gpio.h"

#include <iostream>
#include <thread>
#include <string>
#include <vector>

#define _USE_MATH_DEFINES
#include <cmath>

#define SIDE_LEFT "side_left"
#define SIDE_RIGHT "side_right"
#define ROBOT_SAFE_TURN_CM 5

using namespace std;

class Robot {

public:
	Robot();

	void start();

	void robotLoop();

	void robotDrive(std::vector<Point> instructions);
	void getBaby(Point targetPoint);
	void tossBaby(Point targetPoint);
	void blowCandle(Point targetPoint);
	void spinAndScan();
	void hallwaySweep(Point targetPoint);
	void hallwaySimple(Point targetPoint);
	void goToFrontFromSide(Point targetPoint, string side);
	void leaveRoom();

	void rotateTowards(DoublePoint targetPoint);

private:
    static void signalHandler(int signum);

    static bool done;

    GPIO gpio; //needs to be initialized before all sensors
    thread laserScanInputThread;

    MazeMapper mazeMapper;
    Drive		drive;

    GameState	gameState;
    Point		safeZoneLocation;

    ColorSensor colorSensor;
};

#endif
