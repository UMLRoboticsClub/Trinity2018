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


int computePathLength(vector<Point> deltas) {
	// sum up euclidean distances between delta points and return
	int sum;
	// iterate to size()-1 because the last point doens't have another point to get distance between
	for (int i = 0; i < deltas.size() - 1; i++) {
		// distance between two points
		sum += sqrt(pow(deltas[i + 1].x - deltas[i].x, 2.0) +
			pow(deltas[i + 1].y - deltas[i].y, 2.0));
	}
	// function review: 6/10, "It's ok.."   -IGN
	return sum;
}

//vector<Point> is sequence of waypoints
vector<Point> MazeMapper::findNextTarget(GameState state, robotOps &nextRobotOp) { //only function called by the robot

	//===TEST===============================================
	nextRobotOp = CRADLE;
	vector<Point> testPath = { Point(2,5) };
	return testPath;
	//====================================================

	//check if we have already found an important point where we need to go
	vector<int> primaryTargets = state.getTargetType();
	for (int type : primaryTargets) {
		// if we have a destination in mind
		if (targetPoints[type].size() > 0) {
			// these are for finding the shortest path to the closest known target
			vector<Point> path;
			vector<Point> shortestPath;
			int pathLength = 0;
			int shortestPathLength = 0;
			bool firstPass = true;
			// iterate over all our points associated with this type
			// first point is closest path until another is shorter
			for (int i = 0; i < targetPoints.size(); i++) {
				// AStar path
				path = AStar(targetPoints[type][i]);
				// 'diagonalized' path (optimnal)
				path = optimizePath(path);
				// return-ready vector of deltas
				path = convertToDeltas(path);
				// length of these deltas (we want the shortest length)
				pathLength = computePathLength(path);
				if (firstPass) {
					// on the first pass, closest path is the first path found (obviously)
					shortestPath = path;
					shortestPathLength = pathLength;
					firstPass = false;
				}
				else {
					// compare shortest path to current path and update accordingly
					if (pathLength < shortestPathLength) {
						shortestPath = path;
						shortestPathLength = pathLength;
					}
					// could be 'else if' if you really wanted to be 'efficient' but I don't care. *dabs*
				}
			}
			// we now have the closest path available to us, so we return that
			return shortestPath; // yea boi
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
	vector<Point> moves;
	//distance of target from robot position
	int distVal = distanceField[target.x][target.y];
	vector<Point> neighbors;//used for calculating path
	Point direction(0, 0);  //used for detecting changes in direction, which is recorded as a waypoints
	//until we get back to the robot
	while(distVal != 0){
		neighbors = findOpenNeighbors(target); //grab all neighboring cells
		for(Point neighbor : neighbors){   //find a cell one unit closer to the robot then the current one
			if(distanceField[neighbor.x][neighbor.y] = distVal -1){
				if(neighbor-target != direction){
					//change in direction, add to moves
					moves.push_back(target);
					direction = neighbor-target;
				}
				target = neighbor;
				distVal--;
				break;
			}
		}
	}
	return moves;
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

vector<Point> MazeMapper::AStar(Point target) {
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
	Point startPos = Point(int(robotPos.x), int(robotPos.y));


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
			if (newX < 0 || newX >= occGrid.getWidth() || newY < 0 || newY >= occGrid.getHeight()) continue;

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
	//this function currently can end up with targetLocations occuring twice in a single array.
	//options - ignore that and let the drve people just encounter something twice in a row
	//clear them out at the end
	//specifically ignore them as we're adding.  I choose this one.

	//diagonalize to create shortest possible path
	//grab the starting point, with line to second wayPoint.  Increment end point of that line along the path until hitting a wall, then back one and make that point the second waypoint
	//then repeat from that point.  This is not a perfect optimization.  But it's good enough for now
	vector<Point> optMoves;
	Point startPoint = robotPos;
	Point endPoint = moves[0];
	int movesIndex = 0;
	//for each possible improvement
	while(movesIndex < moves.size()-1){
		Point nextMove = moves[movesIndex+1];
		//fix this to be more robust
		Point direction = (nextMove - endPoint);
		direction.x /= (endPoint.x - nextMove.x + endPoint.y - nextMove.y);
		direction.y /= (endPoint.x - nextMove.x + endPoint.y - nextMove.y);
		while(endPoint != nextMove){//if we reach the next point we have a straight digaonal path to it.
			if(pathIsBlocked(startPoint, endPoint)){//this path is not okay
				endPoint = endPoint - direction;//go back a step, we overshot
				break;
			}
			else{
				endPoint = endPoint - direction;
			}
		}
		//avoid including a waypoint multiple times.  Trust me it could happen otherwise.
		if(optMoves.size() == 0 || endPoint != optMoves[optMoves.size()-1])
			optMoves.push_back(endPoint);

		//done with this path, move on
		startPoint = endPoint;
		endPoint = nextMove;
		movesIndex++;
	}
	//make sure we don't double count the last move which isn't part of the above loop
	if(optMoves.size() == 0 || moves[moves.size()-1] != optMoves[optMoves.size()-1])
		optMoves.push_back(moves[moves.size()-1]);

	return optMoves;
}

bool MazeMapper::pathIsBlocked(Point start, Point end){
	//so I decided to pretend the line was fat as padding so we don't get too close to a wall.
	//literally just iterates along the line and checks the surrounding cells for walls
	//a more efficient alternative would be to make three lines rather than one fat line
	//so offset starting points perpindicular to direction and use those three lines
	//that prevents doublecounting being so much a problem.  I'll make that change.
	//perpindicular is (-x, y) and (x, -y) I think.
	DoublePoint direction;

	float magnitude = sqrt(pow(end.x - start.x, 2) + pow(end.y - start.y, 2));
	direction.x = (end.x-start.x)/magnitude;
	direction.y = (end.y-start.y)/magnitude;
	Point currentCell;
	for (int i = 0; i < magnitude; i ++){
		//iterate along the path
		currentCell.x = static_cast<int>(start.x + direction.x * i);
		currentCell.y = static_cast<int>(start.y + direction.y * i);
		for(int j = -1; j <= 1; j++){
			for(int k = -1; k <= 1; k++){
				 if(occGrid.getValue(currentCell.x + j, currentCell.y + j) == WALL)
				 	return true;
			}
		}
	}
	return false;
}

vector<Point> MazeMapper::convertToDeltas(vector<Point> moves) {
	//moves is originally in form of absolute locations to move to, this fuction converts those to delta locations.
	//literally just returns a vector of moves[i] - moves[i-1]
	for(int i = moves.size()-1; i > 0; i --){
		moves[i] = moves[i] - moves[i-1];
	}
	moves[0] = moves[0] - robotPos;

	return moves;
}

/////////////////////////////

void MazeMapper::laserScanLoop() { //loops updateOccupancyGrid()


	while (true) {
		lidar.getScan();
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

bool MazeMapper::isDiag(int x_offset, int y_offset) {
	return ((x_offset + y_offset + 2) % 2 == 0);
}
