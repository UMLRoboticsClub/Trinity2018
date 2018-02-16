#include "Robot.h"
#include <thread>
#include <iostream>



using namespace std;


// made using this video (threading): https://www.youtube.com/watch?v=LL8wkskDlbs

Robot::Robot() :
	robotPos(), mazeMapper(), robotAngle(0), drive(), gameState(),
	safeZoneLocation(), colorSensor(), IRsensor(), camera()
{
	// variables are initialized through the constructor for now
}

///////////////////////////

void Robot::start(void) {
	// maybe this can go in the constructor in the future
	// Thread dedicated to looping the lazer scanner until the robot dies.
	thread laserScanInputThread(&MazeMapper::laserScanLoop, mazeMapper);
	laserScanInputThread.detach(); // thread should run freely on its own ( this function doesn't wait for it to finish)

								   // Let's start this thing
	robotLoop();
}

///////////////////////////

// I'm not sure when to update these GameState variables..
//bool safeZoneFound;
//bool inRoom;
//bool secondArena;


void Robot::robotLoop(void) {
	// initialize to nothing (doesn't really matter)
	MazeMapper::robotOps nextRobotOperation = MazeMapper::robotOps::NOTHING;
	// location of our target (left as null if no target)
	Point targetPoint;
	// our path variable
	vector<Point> nextPath;

	while (true) {
		// most important line of the program?
		//extPath = mazeMapper.findNextTarget(gameState, nextRobotOperation, targetPoint);
		nextPath = mazeMapper.findNextTarget(gameState, nextRobotOperation);

		// always drive to next location, then do other stuff depending on nextRobotOperation
		robotDrive(nextPath);

		// switch the nextRobotOperation variable and act accordingly
		switch (nextRobotOperation) {
		case MazeMapper::NOTHING:
			// This is here for formality
			break;
		case MazeMapper::CRADLE:
			getBaby(targetPoint);
			break;
		case MazeMapper::SAFEZONE:
			tossBaby(targetPoint);
			break;
		case MazeMapper::EXTINGUISH:
			blowCandle(targetPoint);
			break;
		case MazeMapper::SCANROOM:
			spinAndScan();
			break;
		case MazeMapper::HALLWAY:
			hallwaySweep();
			break;
		}
		// annnnd.. repeat
		break; // without break, code will keep on running forever. Remove this when we start serious testing.
	}
}

void Robot::robotDrive(vector<Point> instructions) {

	for (unsigned int i = 0; i < instructions.size(); i++) {
		drive.drive(instructions[i].x, instructions[i].y);

		// double check position
	}

}

void Robot::getBaby(Point targetPoint) {

	// rotate towards baby and stare into soul
	rotateTowards(targetPoint);

	// extend arms()

	// job well done
	gameState.babyObtained = true;
}
void Robot::tossBaby(Point targetPoint) {


	// rotate towards window
	rotateTowards(targetPoint);


	// murder baby
	// cradleArm.toss() ???



	// job well done
	gameState.babySaved = true;
}
void Robot::blowCandle(Point targetPoint) {


	// face the candle head on
	rotateTowards(targetPoint);



	// blow me
	// extinguisher.extinguish() ??
}
void Robot::spinAndScan(void) {
	//robot will be in appropriate position, so just spin around and get flame and camera data
	drive.rotate(PI); // degrees right?
					  // how do i know when this done?

					  //updating the important points vector as necessary
}
void Robot::hallwaySweep(void) {
	/*
	(potentially) : drive down the hallway, using lidar to detect once we have exited the hallway.
	Then turn the robot so camera is facing back where we came from(so it’ll detect the safezone target)
	then drive the robot sideways through each of the side hallways.Theoretically this should guarantee
	that we find the correct window.
	*/

}

void Robot::rotateTowards(Point target) {
	// convert target to doublepoint for precision, and pass it with robot's position to get angle between them
	double angleBetweenLocations = DoublePoint::computeAngle(robotPos, DoublePoint(target.x, target.y));
	// get angle that we need to rotate in order to face target
	double rotationAngle = angleBetweenLocations - robotAngle;
	// pass to drive
	drive.rotate(rotationAngle);
}