#pragma once

#include <vector>
#include <map>
#include<cmath>

#include "GameState.h"
#include "OccupancyGrid.h"
#include "LaserScanner.h"
#include "Point.h"

using namespace std;

class MazeMapper {
    public:

        // enum of what findNextTarget's return by reference data.
        // (robot operations)
        enum robotOps { NOTHING, CRADLE_SIDE, CRADLE_FRONT, SAFE_ZONE, EXTINGUISH, SCANROOM, EXIT_ROOM, HALLWAY, HALLWAY_SIMPLE, STOP};


        MazeMapper();

        //vector<Point> is sequence of waypoints
        vector<Point> findNextTarget(GameState state, robotOps &nextRobotOp, Point& targetLocation); //only function called by the robot
        robotOps determineRobotOp(int type, GameState& state);
        vector<Point> specialTargetPath(int targetType, vector<Point>& lcoations, int& targetIndex);
        Point closestClearPoint(Point target);
        vector<Point> createTargetPath(Point target);//updates distanceField
        vector<Point> AStar(const Point &target);
        vector<Point> optimizePath(vector<Point>);
        vector<Point> convertToDeltas(vector<Point>);
        bool pathIsBlocked(Point start, Point end);
        bool isDiag(int x_offset, int y_offset);

        void laserScanLoop();
        void updateOccupancyGrid(); //gets laser data and updates grid potentiall have running on interrupt somehow whenever we get a laser scan
        Point computeDistanceField(); //takes gamestate or type of target, called in find
        vector<Point> findOpenNeighbors(Point currentPos);
        double computePathLength(vector<Point> deltas);
    private:
        //map of type of targetPoints to vector of all point of that type


        OccupancyGrid occGrid;
        map<int, vector<Point>> targetPoints;
        vector<vector<int>> distanceField;
        LaserScanner lidar;
};
