#include "robot.h"

#include "logger.h"
#include <thread>
#include <iostream>
#include <csignal>
#include <cstring>

bool Robot::done = false;

void Robot::signalHandler(int signum){
    cout << "\n";
    Logger::log(string(strsignal(signum)) + ", aborting...", Logger::HIGH);
    done = true;

    //remove this soon
    exit(signum);
}

Robot::Robot():
    mazeMapper(), drive(), gameState(),
    safeZoneLocation(), colorSensor(), IRsensor(), camera()
{
    //catch signals to exit safely aka stop the motors when the program is killed
    signal(SIGINT , Robot::signalHandler);
    signal(SIGABRT, Robot::signalHandler);
    signal(SIGFPE , Robot::signalHandler);
    signal(SIGILL , Robot::signalHandler);
    signal(SIGSEGV, Robot::signalHandler);
    signal(SIGTERM, Robot::signalHandler);
    signal(SIGHUP , Robot::signalHandler);

    Logger::log("robot initialized");
    // variables are initialized through the constructor for now
}

///////////////////////////

void Robot::start() {
    // maybe this can go in the constructor in the future
    // Thread dedicated to looping the lazer scanner until the robot dies.
    thread laserScanInputThread(&MazeMapper::laserScanLoop, mazeMapper);
    laserScanInputThread.detach(); // thread should run freely on its own ( this function doesn't wait for it to finish)


    Logger::log("starting robotLoop");
    // Let's start this thing
    robotLoop();
}

///////////////////////////

// I'm not sure when to update these GameState variables..
//bool safeZoneFound;
//bool inRoom;
//bool secondArena;

void Robot::robotLoop() {
    // initialize to nothing (doesn't really matter)
    MazeMapper::robotOps nextRobotOperation = MazeMapper::robotOps::OP_NOTHING;

    // location of our target (left as null if no target)
    Point targetLocation;

    // our path variable
    vector<Point> nextPath;
    //bool done = false;

    while (!done) {
        Logger::log("finding next target");
        // call this bad boy
        nextPath = mazeMapper.findNextTarget(gameState, nextRobotOperation, targetLocation);

        Logger::log("driving to next path");
        // always drive to next location, then do other stuff depending on nextRobotOperation
        robotDrive(nextPath);

        // switch the nextRobotOperation variable and act accordingly
        switch (nextRobotOperation) {
            case MazeMapper::OP_NOTHING:
                // This is here for formality
                break;
            case MazeMapper::OP_CRADLE_FRONT:
                Logger::log("goToSideFromFront");
                goToSideFromFront();
            case MazeMapper::OP_CRADLE_SIDE:
                Logger::log("getBaby");
                getBaby(targetLocation);
                break;
            case MazeMapper::OP_SAFE_ZONE:
                Logger::log("tossBaby");
                tossBaby(targetLocation);
                break;
            case MazeMapper::OP_EXTINGUISH:
                Logger::log("extinguish");
                blowCandle(targetLocation);
                break;
                //differentiation between scanroom and exitroom occurs when door is target,
                //return scan or exit based on whether or not currently in room.
            case MazeMapper::OP_SCANROOM:
                Logger::log("spinAndScan");
                spinAndScan();
                break;
            case MazeMapper::OP_EXIT_ROOM:
                Logger::log("leaveRoom");
                leaveRoom(); //doesn't do the spin move enter room has
                break;
            case MazeMapper::OP_HALLWAY:
                Logger::log("hallwaySweep");
                hallwaySweep();
                break;
            case MazeMapper::OP_HALLWAY_SIMPLE:
                Logger::log("hallwaySimple");
                hallwaySimple();
                break;
            case MazeMapper::OP_STOP:
                Logger::log("success!");
                done = true;
                break;
        }
        // annnnd.. repeat
        break; // without break, code will keep on running forever. Remove this when we start serious testing.
    }
    Logger::log("exiting");
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

void Robot::spinAndScan() {
    //robot will be in appropriate position, so just spin around and get flame and camera data
    //updating the important points vector as necessary
    drive.rotate(M_PI);

    gameState.inRoom = true;
}

void Robot::hallwaySweep() {
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
