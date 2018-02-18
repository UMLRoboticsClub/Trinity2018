#pragma once

#include <vector>
#include <map>
#include <cmath>

#include "gamestate.h"
#include "occupancygrid.h"
#include "laserscanner.h"
#include "point.h"

using namespace std;

class MazeMapper {
    public:

        // enum of what findNextTarget's return by reference data.
        // (robot operations)
        enum robotOps { 
            OP_NOTHING,
            OP_CRADLE_SIDE,
            OP_CRADLE_FRONT,
            OP_SAFE_ZONE,
            OP_EXTINGUISH,
            OP_SCANROOM,
            OP_EXIT_ROOM,
            OP_HALLWAY,
            OP_HALLWAY_SIMPLE,
            OP_STOP
        }; 

        MazeMapper();

        //vector<Point> is sequence of waypoints
        vector<Point> findNextTarget(GameState state, robotOps &nextRobotOp, Point& targetLocation); //only function called by the robot
        robotOps determineRobotOp(int type, GameState& state);
        vector<Point> specialTargetPath(int targetType, vector<Point>& lcoations, int& targetIndex);
        Point closestClearPoint(Point target);
        vector<Point> createTargetPath(Point target);//updates distanceField
        vector<Point> AStar(const Point &target);
        vector<Point> optimizePath(const vector<Point> &moves);
        void convertToDeltas(vector<Point> &moves);
        bool pathIsBlocked(Point start, Point end);
        bool isDiag(int x_offset, int y_offset);

        void laserScanLoop();
        void updateOccupancyGrid(); //gets laser data and updates grid potentiall have running on interrupt somehow whenever we get a laser scan
        Point computeDistanceField(); //takes gamestate or type of target, called in find
        vector<Point> findOpenNeighbors(Point currentPos);
        double computePathLength(const vector<Point> &deltas);
    private:
        //map of type of targetPoints to vector of all point of that type

        OccupancyGrid occGrid;
        map<int, vector<Point>> targetPoints;
        vector<vector<int>> distanceField;
        LaserScanner lidar;
};
