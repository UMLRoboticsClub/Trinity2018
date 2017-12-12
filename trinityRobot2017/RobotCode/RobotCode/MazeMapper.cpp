#include "MazeMapper.h"
#include "globals.h"
#include "constants.h"
#include "Point.h"
#include "Node.h"

#include <vector>
#include <unordered_map>
#include <cmath>
#include <algorithm>

using namespace std;

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



// don't ask.. but AStar algorithm needs it
template <>
struct hash<Point>
{
	std::size_t operator()(const Point& k) const
	{
		using std::size_t;
		using std::hash;
		using std::string;

		// hash the Point object according to it's member variables
		// essential, in order to be unordered_map key
		return ((hash<int>()(k.x)
			^ (hash<int>()(k.y) << 1)) >> 1);
	}
};

vector<Point> AStar(Point target) {
	//implement A* pathfinding algorithm for known points in the known space

	vector<Point> path;							// path to return
	unordered_map<Point, Node*> openNodes;		// map of "open" nodes
	unordered_map<Point, Node*> closedNodes;	// map of "closed" nodes

	Node* parentNode;							// parent node every iteration
	Node* childNode;							// one of four child nodes every iteration
	Node* traverseNode;							// traverse node used to generate path when algorithm finishes

	Point point;								// usefull point object
	Point newNodePoint;							// point object every iteration

	unordered_map<Point, Node*>::iterator mapIterator; // iterator for any unordered_map


	 // directions possible to grow openlist, dg must be 1 as long as their are only 4 directions (up, down, left, right)
	vector<Point> directions = { Point(1, 0), Point(0, 1), Point(-1, 0), Point(0, -1) };
	int dg = 1;
	// newX and newY are the child locations each iteration, heuristic is the new h value for every child
	// and blockValue is the whether the current location is clear or blocked
	int newX = 0, newY = 0, heuristic = 0, blockValue = 0;


	// start where the robot is
	Point startPos = Point(int(robotPos.x), int(robotPos.y);


	// start algorithm with start location in the open list
	point = Point(startPos.x, startPos.y);
	openNodes.insert(pair<Point, Node*>(point, new Node(startPos.x, startPos.y, target.x + target.y)));

	// set parent of first location to NULL (so we can find it later when generating the path)
	openNodes[point]->setParent(NULL, 0);

	// If the target is never found somehow, then this while loop will keep the program from running indefinitely
	while (openNodes.size() > 0) {

		// take node with smallest f ( we use point to find this node, and point will then be key to this node
		mapIterator = openNodes.begin();
		point = mapIterator->first; // node at beginning is our starting lowest f value point
		while (mapIterator != openNodes.end()) {
			// if this node f is smaller than smallest found -> replace
			if (mapIterator->second->getF() < openNodes[point]->getF()) {
				point = mapIterator->first;
			}
			mapIterator++;
		}
		// remove this node from open map and save as parentNode
		parentNode = openNodes[point];
		openNodes.erase(point);
		// put current parent node on closed map
		closedNodes[point] = parentNode;


		// generate children of current node unless they fail to meet requirements (as seen below)
		for (unsigned int i = 0; i < directions.size(); i++) {

			// calculate child node values before making the actual object
			//(so we can test these values to make sure they are good)
			newX = parentNode->getX() + directions[i].x;
			newY = parentNode->getY() + directions[i].y;
			heuristic = abs(target.x - newX) + abs(target.y - newY);
			newNodePoint = Point(newX, newY);

			//if out of bounds, skip this child (with continue statement)
			if (newX < 0 || newX >= occGrid.getTotalWidth() || newY < 0 || newY >= occGrid.getTotalHeight()) continue;

			// if newX and newY are not clear
			// CLEAR, DOOR, EXPLORED_DOOR, HALLWAY - clear, else -> wall
			blockValue = occGrid.getValue(newX, newY);
			// skip if NOT these values
			if (!(blockValue == CLEAR || blockValue == DOOR || blockValue == EXPLORED_DOOR || blockValue == HALLWAY)) continue;


			// if newX and newY are already in the open nodes map
			mapIterator = openNodes.find(newNodePoint);
			if (mapIterator != openNodes.end()) {

				// if new posible g < nodeAlreadyInList.g -> re-parent nodeAlreadyInList so that it's parent is current parentNode
				if (parentNode->getG() + dg < mapIterator->second->getG()) {
					mapIterator->second->setParent(parentNode, dg); // set new parent
					continue;
				}
				// else don't add new node
				else {
					continue;
				}
			}

			// if newX and newY are already in the closed nodes map -> skip
			mapIterator = closedNodes.find(newNodePoint);
			if (mapIterator != closedNodes.end()) {
				continue;
			}

			// creat child, set parent and add to open map
			// heuristic = approximate heuristic using Manhattan Distance ( as opposed to Euclidean Distance or Diagonal Distance)
			childNode = new Node(newX, newY, heuristic);
			childNode->setParent(parentNode, dg);
			openNodes[newNodePoint] = childNode;

			// target acquired
			if (newX == target.x && newY == target.y) {

				// generate vector of path starting from childNode till startNode
				traverseNode = childNode;
				while (traverseNode != NULL) {
					// create Point for path form node x and y
					path.push_back(Point(traverseNode->getX(), traverseNode->getY()));
					traverseNode = traverseNode->getParent();
				}

				//reverse vector (this is faster than inserting the points at the front, which would cause a shift every time
				reverse(path.begin(), path.end());

				// garbage collection -- clear out the open and closed nodes so we get no memory leaks (bad)
				for (mapIterator = openNodes.begin(); mapIterator != openNodes.end(); ++mapIterator) {
					delete mapIterator->second;
				}
				openNodes.clear();
				for (mapIterator = closedNodes.begin(); mapIterator != closedNodes.end(); ++mapIterator) {
					delete mapIterator->second;
				}
				closedNodes.clear();

				//return
				return path;
			}
		}
	}

	// if target is never found, return an empty path :(
	return path;
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
