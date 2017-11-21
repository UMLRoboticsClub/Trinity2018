#include "Robot.h"
#include <thread>

using namespace std;


// made using this video: https://www.youtube.com/watch?v=LL8wkskDlbs

Robot::Robot():
	robotPos(0), robotAngle(0), mazeMapper(), drive(), gameState(),
	safeZoneLocation(), colorSensor(), IRsensor(), camera()
{
	// variables are initialized through the constructor for now
}

///////////////////////////

void Robot::start(void) {
	// maybe this can go in the constructor in the future
	// Thread dedicated to looping the lazer scanner until the robot dies.
	laserScanInputThread(mazeMapper.laserScanLoop);
	laserScanInputThread.detach(); // thread should run freely on its own ( this function doesn't wait for it to finish)
}

///////////////////////////

void Robot::robotLoop(void){

}