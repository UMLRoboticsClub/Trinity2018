#include "robot.h"
#include <thread>
#include <iostream>

Robot::Robot():
	robotPos(GRID_SIZE_CM/2, GRID_SIZE_CM/2), robotAngle(0), mazeMapper(), drive(), gameState(),
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
	MazeMapper::robotOps nextRobotOperation = MazeMapper::robotOps::OP_NOTHING;

	// location of our target (left as null if no target)
	Point targetLocation;

	// our path variable
	vector<Point> nextPath;
	bool done = false;

    while (!done) {
        // call this bad boy
        nextPath = mazeMapper.findNextTarget(gameState, nextRobotOperation, targetLocation);
        // always drive to next location, then do other stuff depending on nextRobotOperation
        robotDrive(nextPath);

        // switch the nextRobotOperation variable and act accordingly
        switch (nextRobotOperation) {
            case MazeMapper::OP_NOTHING:
                // This is here for formality
                break;
            case MazeMapper::OP_CRADLE_FRONT:
                goToSideFromFront();
            case MazeMapper::OP_CRADLE_SIDE:
                getBaby(targetLocation);
                break;
            case MazeMapper::OP_SAFE_ZONE:
                tossBaby(targetLocation);
                break;
            case MazeMapper::OP_EXTINGUISH:
                blowCandle(targetLocation);
                break;
                //differentiation between scanroom and exitroom occurs when door is target,
                //return scan or exit based on whether or not currently in room.
            case MazeMapper::OP_SCANROOM:
                spinAndScan();
                break;
            case MazeMapper::OP_EXIT_ROOM:
                leaveRoom(); //doesn't do the spin move enter room has
                break;
            case MazeMapper::OP_HALLWAY:
                hallwaySweep();
                break;
            case MazeMapper::OP_HALLWAY_SIMPLE:
                hallwaySimple();
                break;
            case MazeMapper::OP_STOP:
                done = true;
                break;
        }
        // annnnd.. repeat
        break; // without break, code will keep on running forever. Remove this when we start serious testing.
    }
}

void Robot::hallwaySimple(){

}

void Robot::leaveRoom(){

    gameState.inRoom = false;
}

void Robot::goToSideFromFront(){

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

    // kidnapBaby.exe

    // job well done
    gameState.babyObtained = true;
}

void Robot::tossBaby(Point targetPoint) {
    // rotate towards window
    rotateTowards(targetPoint);

    // "save baby".exe

    // job well done
    gameState.babySaved = true;
}

void Robot::blowCandle(Point targetPoint) {
    // face the candle head on
    rotateTowards(targetPoint);

    // blow me
    // extinguisher.extinguish() ??

    // another one down
    gameState.numCandlesExtinguished++;
}

void Robot::spinAndScan(void) {
    //robot will be in appropriate position, so just spin around and get flame and camera data
    //updating the important points vector as necessary
    drive.rotate(M_PI);

    gameState.inRoom = true;
}

void Robot::hallwaySweep(void) {
    /*
       (potentially) : drive down the hallway, using lidar to detect once we have exited the hallway.
       Then turn the robot so camera is facing back where we came from(so it'¿½ll detect the safezone target)
       then drive the robot sideways through each of the side hallways.Theoretically this should guarantee
       that we find the correct window.
       */

}

void Robot::rotateTowards(Point target) {
    // convert target to doublepoint for precision, and pass it with robot's position to get angle between them
    //////
    // heads up, this line computes the angle between int and double points, remove this if it's ok
    /////
    double angleBetweenLocations = computeAngle(robotPos, DoublePoint(target.x, target.y));
    // get angle that we need to rotate in order to face target
    double rotationAngle = angleBetweenLocations - robotAngle;
    // pass to drive
    drive.rotate(rotationAngle);
}
