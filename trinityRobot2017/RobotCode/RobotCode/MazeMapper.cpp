#include "MazeMapper.h"
#include "globals.h"
#include "constants.h"
	//constructors
MazeMapper::MazeMapper() {}

///////////////////////////

//vector<Point> is sequence of waypoints
vector<Point> MazeMapper::findNextTarget(GameState state) { //only function called by the robot
	//check if we have already found an important point where we need to go
	vector<int> primaryTargets = state.getTargetType();

	//reverse this loop it's dumb this way
	for (int type: primaryTargets) {
		for (auto foundTarget : targetPoints) {
			if (foundTarget.first == type) {
				return AStar(foundTarget.second);
			}
		}
	}

	//no important points already found, go to distance field and find an unknown
	Point target = computeDistanceField();
	//list of solely UDLR directional movements
	vector<Point> moves = createTargetPath(target);
	//optimizes with direct diagonalized motion
	moves = optimizePath(moves);
	return convertToDeltas(moves);
}

/////////////////////////

vector<Point> MazeMapper::createTargetPath(Point target) {//distance field already created
	//start at target which has a specific val in distancie field, locate neighbor with distVal 1 less, repeat.
	//when the direction changes, push the current point into the moves list.
	
}

vector<Point> MazeMapper::AStar(Point target) {
	//implement A* pathfinding algorithm for known points in the known space
}

vector<Point> MazeMapper::optimizePath(vector<Point> moves) {
	//diagonalize to create shortest possible paths
}

vector<Point> MazeMapper::convertToDeltas(vector<Point> moves) {
	//moves is originally in form of absolute locations to move to, this fuction converts those to delta locations.
}

/////////////////////////////

void MazeMapper::laserScanLoop() { //loops updateOccupancyGrid()


	while (true) {
		LaserScanner.getScan();
		// We will figure out how often this should be called later
		updateOccupancyGrid();
	}
}

void MazeMapper::updateOccupancyGrid(){ //gets laser data and updates grid potentially have running on interrupt somehow whenever we get a laser scan
	LaserScan scan = lidar.getLaserScan();
	
	// we can change element.getAngle() and element.getDist() but we're using these for now
	for(auto element : scan){ //for each element in the scan 
		// given the angle and distance...
		//everything between robotPos and newPos is clear
		for(int i = 0; i < element.getDist() - 1; i++){ 
			float xDist = i * cos(element.getAngle());
			float YDist = i * sin(element.getAngle());
			occGrid.update(robotPos.x + xDist, robotPos.y + yDist, CLEAR);
		}
		
		float xDist = element.getDist() * cos(element.getAngle());
		float yDist = element.getDist() * sin(element.getAngle());
		
		occGrid.update(robotPos.x + xDist, robotPos.y + yDist, WALL);
	}
	
	// call find doors & hallways, which will update important values
	
	// call find flame, which will update important values
	
	// now that we have all important values for this scan, we can update the occ grid
	for(auto element : targetPoints){ //element.first is the key, element.second is the value
		for(int i = 0; i < element.second.size(); i++){
			occGrid.update(element.second[i].x, element.second[i].y, element.first);
		}
	}
}

/////////////////////////////
Point MazeMapper::computeDistanceField() { //takes type of target, called in find 
	//determine appropriate items to look for
	vector<Point> boundary;
	boundary.push_back(robotPos);
	vector<Point> neighbors;
	Point currentPoint;
	while (!boundary.empty()) {
		Point currentPoint = boundary.front();
		neighbors = findOpenNeighbors(currentPoint);
		for (Point neighbor : neighbors) {
			if (occGrid.getValue(neighbor) == -1)
				return neighbor;
			boundary.push_back(neighbor);
		}
	}
	return Point(-1, -1);
}

vector<Point> MazeMapper::findOpenNeighbors(Point currentPos) {
	std::vector< Point > openNeighbors;
	for (int x_offset = -1; x_offset < 2; x_offset++) {
		for (int y_offset = -1; y_offset < 2; y_offset++) {
			if (!isDiag(x_offset, y_offset) && \
				occGrid.getValue(currentPos.x + x_offset, currentPos.y + y_offset) <= CLEAR_THRESHOLD &&  occGrid.getValue(currentPos.x + x_offset, currentPos.y + y_offset) >= 0) {
				openNeighbors.push_back(Point(currentPos.x + x_offset, currentPos.y + y_offset));
			}
		}
	}
	return openNeighbors;
}

bool isDiag(int x_offset, int y_offset) {
	return ((x_offset + y_offset + 2) % 2 == 0);
}
