#ifndef ROBOT_H
#define ROBOT_H

#include "point.h"
#include "mazemapper.h"
#include "gamestate.h"
#include "constants.h"
#include "drive.h"
#include "colorsensor.h"
#include "irsensor.h"
#include "camera.h"
#include "gpio.h"

#include <iostream>
#include <thread>

using namespace std;

class Robot {

public:
	Robot();

	void start();

	void robotLoop();

	void robotDrive(vector<Point> instructions);
	void getBaby(Point targetPoint);
	void tossBaby(Point targetPoint);
	void blowCandle(Point targetPoint);
	void spinAndScan();
	void hallwaySweep();
	void hallwaySimple();
	void goToSideFromFront();
	void leaveRoom();

	void rotateTowards(Point target);

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
    IRSensor	IRsensor;
    Camera		camera;
};

#endif
