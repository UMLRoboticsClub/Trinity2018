#pragma once
#include<vector>
#include "GameState.h"
#include "occupancyGrid.h"
#include "LaserScanner.h"
#include "Point.h"
#include <map>
using namespace std;
class MazeMapper
{
public:
	MazeMapper();

	//vector<Point> is sequence of waypoints
	vector<Point> findNextTarget(GameState state); //only function called by the robot
	vector<Point> createTargetPath(Point target);//updates distanceField
	vector<Point> AStar(Point target);
	vector<Point> optimizePath(vector<Point>);
	vector<Point> convertToDeltas(vector<Point>);
	bool pathIsBlocked(Point start, Point end);
	bool isDiag(int x_offset, int y_offset);

	void laserScanLoop();
	void updateOccupancyGrid(); //gets laser data and updates grid potentiall have running on interrupt somehow whenever we get a laser scan
	Point computeDistanceField(); //takes gamestate or type of target, called in find
	vector<Point> findOpenNeighbors(Point currentPos);
private:
	//map of type of targetPoints to vector of all point of that type
	map<int, vector<Point>> targetPoints;
	OccupancyGrid occGrid;
	vector<vector<int>> distanceField;
	LaserScanner lidar;
};
