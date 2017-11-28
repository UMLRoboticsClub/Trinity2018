#pragma once
#include<vector>
#include "GameState.h"
#include "occupancyGrid.h"
#include "LaserScanner.h"
using namespace std;
class MazeMapper
{
public:
	MazeMapper();

	//vector<Point> is sequence of waypoints
	vector<Point> findNextTarget(GameState state); //only function called by the robot
	void createTargetPath();//updates distanceField

	void laserScanLoop();
	void updateOccupancyGrid(); //gets laser data and updates grid potentiall have running on interrupt somehow whenever we get a laser scan
	void computeDistanceField(GameState state); //takes gamestate or type of target, called in find 

private:
	OccupancyGrid occGrid;
	vector<vector<int>> distanceField;
	LaserScanner lidar;
};

