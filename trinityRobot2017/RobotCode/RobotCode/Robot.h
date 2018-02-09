#ifndef ROBOT_H
#define ROBOT_H

#include "Point.h"
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
	void getBaby(void);
	void tossBaby(void);
	void blowCandle(void);
	void spinAndScan(void);
	void hallwaySweep(void);

private:

	thread laserScanInputThread;
	Point		robotPos;
	float		robotAngle;

	MazeMapper	mazeMapper;
	Drive		drive;

	GameState	gameState;
	Point		safeZoneLocation;

	ColorSensor colorSensor;
	IRSensor	IRsensor;
	Camera		camera;
};


#endif
