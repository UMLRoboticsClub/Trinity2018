#include "MazeMapper.h"

class MazeMapper {
	//constructors

	MazeMapper();

	///////////////////////////

	//vector<Point> is sequence of waypoints
	vector<Point> findNextTarget(GameState state) { //only function called by the robot

	}

	/////////////////////////

	void createTargetPath() {//updates distanceField

	}

	/////////////////////////////

	void updateOccupancyGrid(){ //gets laser data and updates grid potentially have running on interrupt somehow whenever we get a laser scan

	}

	/////////////////////////////

	void computeDistanceField(GameState state){ //takes gamestate or type of target, called in find 
	
	}
};