#ifndef ROBOT_H
#define ROBOT_H

#include "Point.h"
#include "DoublePoint.h"
#include "MazeMapper.h"
#include "GameState.h"
#include "Constants.h"
#include "Drive.h"
#include "ColorSensor.h"
#include "IRSensor.h"
#include "Camera.h"
#include <iostream>
#include <thread>


using namespace std;

class Robot {

public:
	Robot();

	void start(void);

	void robotLoop(void);

	void robotDrive(vector<Point> instructions);
	void getBaby(Point targetPoint);
	void tossBaby(Point targetPoint);
	void blowCandle(Point targetPoint);
	void spinAndScan(void);
	void hallwaySweep(void);

	void rotateTowards(Point target);

private:

	thread laserScanInputThread;
	DoublePoint	robotPos;
	double		robotAngle;

	MazeMapper	mazeMapper;
	Drive		drive;

	GameState	gameState;
	Point		safeZoneLocation;

	ColorSensor colorSensor;
	IRSensor		IRsensor;
	Camera		camera;

	const double PI = 3.14159265358979;
};


#endif
