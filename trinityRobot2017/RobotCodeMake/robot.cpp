#include "robot.h"
#include "point.h"
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
    safeZoneLocation(), colorSensor()// IRsensor()//, camera()
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
    std::vector<Point> nextPath;
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
            case MazeMapper::OP_CRADLE_LEFT:
                Logger::log("goToFrontFromLeft");
                goToFrontFromSide(targetLocation, SIDE_LEFT);
                break;
            case MazeMapper::OP_CRADLE_RIGHT:
                Logger::log("goToFrontFromRight");
                goToFrontFromSide(targetLocation, SIDE_RIGHT);
                break;
            case MazeMapper::OP_CRADLE_FRONT:
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
                hallwaySweep(targetLocation);
                break;
            case MazeMapper::OP_HALLWAY_SIMPLE:
                Logger::log("hallwaySimple");
                hallwaySimple(targetLocation);
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

void Robot::hallwaySimple(Point targetPoint){
    gameState.secondArena = false;
}

void Robot::leaveRoom(){

    gameState.inRoom = false;
}

void Robot::goToFrontFromSide(Point targetPoint, string side){
    
    // orientate ourselves
    Point closestClear = mazeMapper.closestClearPoint(targetPoint);

    // get angle between side of cradle and the closest clear point
    double angleBetween = computeAngle(targetPoint, closestClear);
    
    // center is opposite way
    double pointToCenter = angleBetween + M_PI;

    // add or subtract 90 degrees to get diretion that points to the front of cradle
    double pointToFront = 0;
    if(side == SIDE_LEFT){ 
        pointToFront = angleBetween + (M_PI/2.0);
    }else if(side == SIDE_RIGHT){
        pointToFront = angleBetween - (M_PI/2.0);
    }

    // safeDistance is for turning, then we get to correct distance afterward
    double cradleHalf = CRADLE_SIZE_CM / 2.0;
    double robotHalf = ROBOT_DIAMETER_CM / 2.0;
    double safeDistance = robotHalf + ROBOT_SAFE_TURN_CM;
    
    // calculate front of cradle by adding these two vector scaled by the cradle's size
    DoublePoint cradleFront = DoublePoint(cradleHalf * cos(pointToCenter), cradleHalf * sin(pointToCenter)) + DoublePoint(cradleHalf * cos(pointToFront), cradleHalf * sin(pointToFront));

    // these are rounded just to be safe
    // robot safe rotate distance vector
    Point safeRotatePoint( round(cradleFront.x + safeDistance * cos(pointToFront)), round(cradleFront.y + safeDistance * sin(pointToFront)) );
    
    // robot 'touching cradle vector'
    Point touchCradlePoint( round(cradleFront.x + robotHalf * cos(pointToFront)), round(cradleFront.y + robotHalf * sin(pointToFront)) );

    // astar returns path to safe point and then we optimize and we then drive the path
    robotDrive(mazeMapper.optimizePath(mazeMapper.AStar(safeRotatePoint)));
     
    // now we rotate towards the cradle
    rotateTowards(cradleFront);

    //now we get up real close
    robotDrive(mazeMapper.optimizePath(mazeMapper.AStar(touchCradlePoint)));
    
    // SNATCH_BABY()
}

void Robot::robotDrive(std::vector<Point> instructions) {

    for (unsigned int i = 0; i < instructions.size(); i++) {
        drive.drive(instructions[i]);

        // double check position
    }

}

void Robot::getBaby(Point targetPoint) {
    // rotate towards baby and stare into soul
    rotateTowards(DoublePoint(targetPoint));

    // kidnapBaby.exe

    // job well done
    gameState.babyObtained = true;
}

void Robot::tossBaby(Point targetPoint) {
    // rotate towards window
    rotateTowards(DoublePoint(targetPoint));

    // "save baby".exe

    // job well done
    gameState.babySaved = true;
}

void Robot::blowCandle(Point targetPoint) {
    // face the candle head on
    rotateTowards(DoublePoint(targetPoint));

    // blow me
    gpio_write(0, solenoidPin, 1);
    time_sleep(2);
    gpio_write(0, solenoidPin, 0);
    // extinguisher.extinguish() ??

    // another one down
    gameState.numCandlesExtinguished++;
}

void Robot::spinAndScan() {
    //robot will be in appropriate position, so just spin around and get flame and camera data
    //updating the important points vector as necessary
    drive.rotate(2 * M_PI);

    gameState.inRoom = true;
}

void Robot::hallwaySweep(Point targetPoint) {
    /*
       (potentially) : drive down the hallway, using lidar to detect once we have exited the hallway.
       Then turn the robot so camera is facing back where we came from(so it'¿½ll detect the safezone target)
       then drive the robot sideways through each of the side hallways.Theoretically this should guarantee
       that we find the correct window.
    */

    rotateTowards(DoublePoint(targetPoint));
    //now we need to move "left" and "right" which is done via robotAngle

    //so we need to find which direction the hallway is in  this happens in MazeMapper!  targetLoc
    //then we need to move down it until we are no longer in the hallway.  can we see the end of it?  
    //if we can see the far end of the arena we can just subtract for distance to move
    //move that far, and we also have direction which tells us which way to face
    //then strafe sideways set distance and check camera
    //when camera tells us we saw the thing, record location as safeZonelocation.
    
    // get angles in directions we will patrol
    double leftAngle = getRobotAngle() + (M_PI_2/2);
    double rightAngle = getRobotAngle() - (M_PI_2/2);
    
    double patrolLength = ((ARENA_LENGTH_CM / 2) + ROBOT_DIAMETER_CM);

    // calculate what patrol points are
    int leftX = getRobotPos().x + patrolLength * cos(leftAngle);
    int leftY = getRobotPos().y + patrolLength * sin(leftAngle);
    
    int rightX = getRobotPos().x + patrolLength * cos(rightAngle);
    int rightY = getRobotPos().y + patrolLength * sin(rightAngle);

    // first patrol
    drive.drive(DoublePoint(leftX, leftY));
    //double check position? (maybe we need a funciton like checkPosition() )?? 
    
    // vision.scan()

    // second patrol
    drive.drive(DoublePoint(rightX, rightY));
    // double check position

    // vision.second_scan()

    // update booleans
    gameState.secondArena = true;
    gameState.safeZoneFound = true;
}

void Robot::rotateTowards(DoublePoint targetPoint) {
    // convert target to doublepoint for precision, and pass it with robot's position to get angle between them
    //////
    // heads up, this line computes the angle between int and double points, remove this if it's ok
    /////
    double angleBetweenLocations = computeAngle(getRobotPos(), DoublePoint(targetPoint.x, targetPoint.y));
    // get angle that we need to rotate in order to face target
    double rotationAngle = angleBetweenLocations - getRobotAngle();
    // pass to drive
    drive.rotate(rotationAngle);
}
