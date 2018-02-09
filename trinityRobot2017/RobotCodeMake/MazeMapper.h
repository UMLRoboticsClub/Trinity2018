#pragma once

#include <vector>
#include <map>

#include "GameState.h"
#include "OccupancyGrid.h"
#include "LaserScanner.h"
#include "Point.h"

using namespace std;

class MazeMapper {
    public:

        // enum of what findNextTarget's return by reference data.
        // (robot operations)
        enum robotOps { NOTHING, CRADLE_SIDE, CRADLE_FRONT, DROP_BABY, SAFEZONE, EXTINGUISH, SCANROOM, EXITROOM, HALLWAY, HALLWAY_SIMPLE, STOP};


        MazeMapper();

        //vector<Point> is sequence of waypoints
        vector<Point> findNextTarget(GameState state, robotOps &nextRobotOp); //only function called by the robot
        robotOps determineRobotOp(int type, GameState state, Point targetLoc);
        vector<Point> specialTargetPath(int targetType, vector<Point>& lcoations, int& targetindex);
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
        int computePathLength(vector<Point> deltas);
    private:
        //map of type of targetPoints to vector of all point of that type

        map<int, vector<Point>> targetPoints;
        OccupancyGrid occGrid;
        vector<vector<int>> distanceField;
        LaserScanner lidar;
};
