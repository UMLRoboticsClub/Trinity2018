#include "MazeMapper.h"
#include "Point.h"
	//constructors

MazeMapper::MazeMapper() {}

	///////////////////////////

	//vector<Point> is sequence of waypoints
	vector<Point> MazeMapper::findNextTarget(GameState state) { //only function called by the robot

	}

	/////////////////////////

	void MazeMapper::createTargetPath() {//updates distanceField

	}

	/////////////////////////////

	void MazeMapper::updateOccupancyGrid(){ //gets laser data and updates grid potentially have running on interrupt somehow whenever we get a laser scan

	}

	/////////////////////////////

	void MazeMapper::computeDistanceField(GameState state){ //takes gamestate or type of target, called in find 
		
	}