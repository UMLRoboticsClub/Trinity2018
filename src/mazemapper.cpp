#include "mazemapper.h"
#include "logger.h"
#include "globals.h"
#include "constants.h"
#include "point.h"
#include "node.h"
#include "robot.h"

#include <vector>
#include <unordered_map>
#include <cmath>
#include <algorithm>
#include <functional>
#include <iomanip>

//constructors
MazeMapper::MazeMapper(): occGrid(), targetPoints(){//, lidar(){
    distanceField = vector<vector<int>>(GRID_SIZE_CELLS);
    for(unsigned i = 0; i < distanceField.size(); ++i){
        distanceField[i] = vector<int>(GRID_SIZE_CELLS);
        for(unsigned j = 0; j < distanceField[i].size(); ++j){
            distanceField[i][j] = -1;
        }
    }
}

MazeMapper::~MazeMapper(){
    Logger::log("MazeMapper quitting");
}

///////////////////////////

//Precondition:  deltas is a vector of delta positions, not absolute positions
//PostCondition: returns the total path length of the path represented by deltas
double MazeMapper::computePathLength(const vector<Point> &deltas) {
    // sum up euclidean distances between delta points and return
    double sum = 0;
    // iterate to size()-1 because the last point doesn't have another point to get distance between
    for (unsigned i = 0; i < deltas.size(); ++i) {
        //compute magnitude of each delta
        sum += sqrt(pow(deltas[i].x, 2) + pow(deltas[i].y, 2));
    }
    // function review: 6/10, "It's ok.."   -IGN
    return sum;
}

//vector<Point> is sequence of waypoints
vector<Point> MazeMapper::findNextTarget(GameState &state, robotOps &nextRobotOp, Point& targetLocation) { //only function called by the robot
    if(state.getTargetType() == START_ZONE){
        vector<Point> path = AStar(Point(610, 610));
        path = optimizePath(path);
        nextRobotOp = OP_STOP;
        return path;
    }
    nextRobotOp = OP_NOTHING;
    for(unsigned i = 0; i < distanceField.size(); ++i){
        for(unsigned j = 0; j < distanceField[i].size(); j ++){
            distanceField[i][j] = -1;
        }
    }
    //===TEST===============================================
    //nextRobotOp = CRADLE;
    //vector<Point> testPath = { Point(2,5) };
    //return testPath;
    //====================================================

    //check if we have already found an important point where we need to go
    vector<int> primaryTargets = state.getTargetType();
    for (const int type : primaryTargets) {
        // if we have a destination in mind
        if (getTargetPointsOfType(type).size() > 0) {
            //update robotOps
            nextRobotOp = determineRobotOp(type, state);

            //compute the actual path
            int targetIndex = 0;
            std::vector<Point> path = specialTargetPath(type, getTargetPointsOfType(type), targetIndex, targetLocation); //in here is when we actually determine the target, so this would be the place, or to have it return something and make this grosser
            //getting testing to work
            if(type == DOOR)
                targetPoints[EXPLORED_DOOR].push_back(getTargetPointsOfType(type)[targetIndex]);
            if(type == FLAME)
                targetPoints[EXTINGUISHED].push_back(getTargetPointsOfType(type)[targetIndex]);
            //targetLocation = targetPoints[type][targetIndex];//move this to inside specialTargetPath function
            //targetPoints[type].erase(targetPoints[type].begin() + targetIndex);
            removeTargetPoint(type, targetIndex);
            return path;
            //this whole block could be in a separate function.  And so it shall be
            // these are for finding the shortest path to the closest known target
            /*
               vector<Point> path;
               vector<Point> shortestPath;
               int pathLength = 0;
               int shortestPathLength = 0;
               bool firstPass = true;
            // iterate over all our points associated with this type
            // first point is closest path until another is shorter
            for (int i = 0; i < targetPoints[type].size(); ++i) {
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
            */
        }
    }

    //no important points already found, go to distance field and find an unknown
    targetLocation = computeDistanceField();

    //list of solely UDLR directional movements
    //this somehow came out uninitialized.  wtf.
    vector<Point> moves = createTargetPath(targetLocation);
    cout << "move 1: " << moves[0].x << " " << moves[0].y << endl;
    //optimizes with direct diagonalized motion
    moves = optimizePath(moves);
    cout << "move 2: " << moves[0].x << " " << moves[0].y << endl;
    //reduce last move by robotRadius, or to zero.
    Point lastMove = moves[moves.size()-1];
    Point prevMove = moves.size() > 1 ? moves[moves.size()-2] : Point(getRobotPos());
    Point delta = lastMove - prevMove;
    double mag = sqrt(pow(lastMove.x - prevMove.x, 2) + pow(lastMove.y - prevMove.y, 2));
    double mult = mag - ROBOT_DIAMETER_CM/2 < 0 ? 0 : mag - ROBOT_DIAMETER_CM/2;
    delta *= mult;
    lastMove = prevMove + delta;
    cout << "move 3 " << moves[0].x << " " << moves[0].y << endl;
    //convertToDeltas(moves);
    cout << "move 4: " << moves[0].x << " " << moves[0].y << endl;
    return moves;
}

MazeMapper::robotOps MazeMapper::determineRobotOp(int type, GameState& state){
    switch(type){
        case START_ZONE:
            return OP_STOP;
        case HALLWAY:
            return state.secondArena ? OP_HALLWAY_SIMPLE : OP_HALLWAY;
        case FLAME:
            return OP_EXTINGUISH;
        case CANDLE:
            return OP_EXTINGUISH;
        case DOOR:
            return OP_SCANROOM;
        case EXPLORED_DOOR:
            return OP_EXIT_ROOM;
        case SAFE_ZONE:
            return OP_SAFE_ZONE;
        case FRONT_SIDE_CRADLE:
            return OP_CRADLE_FRONT;
        case LEFT_SIDE_CRADLE:
            return OP_CRADLE_LEFT;
        case RIGHT_SIDE_CRADLE:
            return OP_CRADLE_RIGHT;
        default:
            return OP_NOTHING;
    }
}

vector<Point> MazeMapper::specialTargetPath(int targetType, vector<Point> locations, int& targetIndex, Point& targetLocation){
    // these are for finding the shortest path to the closest known target
    vector<Point> path;
    vector<Point> shortestPath;
    int shortestPathLength = 0;
    bool firstPass = true;
    Point currentTargetLoc;
    // iterate over all our points associated with this type
    // first point is closest path until another is shorter

    for (unsigned int i = 0; i < locations.size(); ++i) {
        currentTargetLoc = locations[i];

        if(targetType == FLAME
                || targetType == CANDLE
                || targetType == LEFT_SIDE_CRADLE
                || targetType == RIGHT_SIDE_CRADLE
                || targetType == FRONT_SIDE_CRADLE
                || targetType == SAFE_ZONE){
            locations[i] = closestClearPoint(locations[i]);        
        }
        else if(targetType == HALLWAY){

            //we need to find the actual end of the hallway.  Look in all directions from the target, find furthest.  Should work.  Ish.  That's not really how that works.  
            //instead find shortest direction, then check both directions perpindicular to that.  
            //guees that needs to be a distance Field?  but no.  we need more circular to properly detect it.  do a circle thing.  width of hallway.  
            //iterate through angles, only searching as far as half the width of the hallway.  There we go.
            double directionAngle = 0;
            double shortestDistance = HALLWAY_WIDTH_CM/2;
            for(double k = 0; k < 2*M_PI; k += M_PI/32){
                for(int j = 0; j < shortestDistance; j ++){
                    if(occGrid.getValue(locations[i] + Point(j * cos(k), j * sin(k))) == WALL){
                        shortestDistance = j;
                        directionAngle = k;
                    }
                }
            }
            //now we have our direction, coolio.
            DoublePoint direction(cos(directionAngle), sin(directionAngle));
            DoublePoint perp1(-direction.y, direction.x);
            DoublePoint perp2(direction.y, -direction.x);

            int distance1 = 0;
            int distance2 = 0;

            DoublePoint looking = locations[i];
            while(occGrid.getValue(Point(looking)) == CLEAR){
                looking += perp1;
                ++distance1;
            }
            looking = locations[i];
            while(occGrid.getValue(Point(looking)) == CLEAR){
                looking += perp2;
                ++distance2;
            }
            double dist = 0;
            if(distance1 > distance2){
                dist = distance1;
                direction = perp1;
            }
            targetLocation = locations[i];

            //we know which direction and distance the far wall is.
            dist -= ARENA_LENGTH_CM;//now we have the point for which the 
            dist += ROBOT_DIAMETER_CM/2 + 2;//puts robot actually centered in the hallway
            locations[i] += Point(dist*direction.x, dist*direction.y);

            //we need to get the target location out of here.  pass by ref again?
        }

        // AStar path
        path = AStar(locations[i]);

        // 'diagonalized' path (optimnal)
        path = optimizePath(path);

        // return-ready vector of deltas
        convertToDeltas(path);

        // length of these deltas (we want the shortest length)
        int pathLength = computePathLength(path);
        if (firstPass) {
            // on the first pass, closest path is the first path found (obviously)
            shortestPath = path;
            shortestPathLength = pathLength;
            firstPass = false;
            targetIndex = 0;
            targetLocation = currentTargetLoc;
        } else {
            // compare shortest path to current path and update accordingly
            if (pathLength < shortestPathLength) {
                shortestPath = path;
                shortestPathLength = pathLength;
                targetIndex = i;
                targetLocation = currentTargetLoc;
            }
            // could be 'else if' if you really wanted to be 'efficient' but I don't care. *dabs*
        }
    }
    // we now have the closest path available to us, so we return that
    return shortestPath; // yea boi
}

//doesn't need a full distancefield, because the solid around it is square
Point MazeMapper::closestClearPoint(const Point &target){
    for(int i = 0 ;; ++i){
        if(occGrid.getValue(target + Point(0, i))  == CLEAR) 
            return target + Point(0, i);
        if(occGrid.getValue(target + Point(0, -i)) == CLEAR)
            return target + Point(0, -i);
        if(occGrid.getValue(target + Point(i, 0))  == CLEAR)
            return target + Point(i, 0);
        if(occGrid.getValue(target + Point(-i, 0)) == CLEAR)
            return target + Point(-i, 0);
    }
}

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
            if(distanceField[neighbor.x][neighbor.y] == distVal -1){
                if(neighbor-target != direction || sqrt(pow(target.x - moves[moves.size()-1].x, 2) + pow(target.y - moves[moves.size()-1].y, 2)) > 50){
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
    std::reverse(moves.begin(), moves.end());
    return moves;
}

// don't ask.. but AStar algorithm needs it
namespace std {
    template <>
        struct hash<Point> {
            size_t operator()(const Point& k) const {
                // hash the Point object according to it's member variables
                // essential, in order to be unordered_map key
                return ((hash<int>()(k.x) ^ (hash<int>()(k.y) << 1)) >> 1);
            }
        };
}

vector<Point> MazeMapper::AStar(const Point &target) {

    //implement A* pathfinding algorithm for known points in the known space

    vector<Point> path;							// path to return
    unordered_map<Point, Node*> openNodes;		// map of "open" nodes
    unordered_map<Point, Node*> closedNodes;	// map of "closed" nodes

    Node* parentNode;							// parent node every iteration
    Node* childNode;							// one of four child nodes every iteration
    Node* traverseNode;							// traverse node used to generate path when algorithm finishes

    Point newNodePoint;							// point object every iteration

    // directions possible to grow openlist, dg must be 1 as long as their are only 4 directions (up, down, left, right)
    vector<Point> directions = { Point(1, 0), Point(0, 1), Point(-1, 0), Point(0, -1) };
    int dg = 1;
    // newX and newY are the child locations each iteration, heuristic is the new h value for every child
    // and blockValue is the whether the current location is clear or blocked
    int newX = 0, newY = 0, heuristic = 0;

    // start algorithm with start location in the open list
    Point point(getRobotPos());
    openNodes.insert(pair<Point, Node*>(point, new Node(getRobotPos().x, getRobotPos().y, target.x + target.y)));

    // set parent of first location to NULL (so we can find it later when generating the path)
    openNodes[point]->setParent(NULL, 0);

    // If the target is never found somehow, then this while loop will keep the program from running indefinitely
    while (!openNodes.empty()) {

        // take node with smallest f ( we use point to find this node, and point will then be key to this node
        auto iter = openNodes.begin();
        point = iter->first; // node at beginning is our starting lowest f value point
        while (iter != openNodes.end()) {
            // if this node f is smaller than smallest found -> replace
            if (iter->second->f < openNodes[point]->f) {
                point = iter->first;
            }
            ++iter;
        }

        // remove this node from open map and save as parentNode
        parentNode = openNodes[point];

        openNodes.erase(point);
        // put current parent node on closed map
        closedNodes[point] = parentNode;

        // generate children of current node unless they fail to meet requirements (as seen below)
        for (unsigned int i = 0; i < directions.size(); ++i) {

            // calculate child node values before making the actual object
            //(so we can test these values to make sure they are good)

            newX = parentNode->x + directions[i].x;
            newY = parentNode->y + directions[i].y;

            heuristic = abs(target.x - newX) + abs(target.y - newY);
            newNodePoint = Point(newX, newY);

            //if out of bounds, skip this child (with continue statement)
            if (newX < 0 || newX >= occGrid.size || newY < 0 || newY >= occGrid.size){
                continue;
            }

            // if newX and newY are not clear
            // CLEAR, DOOR, EXPLORED_DOOR, HALLWAY - clear, else -> wall

            // skip if NOT these values
            switch(occGrid.getValue(newX, newY)){
                case CLEAR:
                case DOOR:
                case EXPLORED_DOOR:
                case HALLWAY:
                    break;
                default:
                    continue;
            }

            {
                auto iter = openNodes.find(newNodePoint);
                // if newX and newY are already in the open nodes map
                if (iter != openNodes.end()) {
                    // if new posible g < nodeAlreadyInList.g -> re-parent 
                    //nodeAlreadyInList so that it's parent is current parentNode
                    if (parentNode->g + dg < iter->second->x) {
                        iter->second->setParent(parentNode, dg); // set new parent
                    }
                    continue;
                }
            }

            // if newX and newY are already in the closed nodes map -> skip
            if (closedNodes.find(newNodePoint) != closedNodes.end()) {
                continue;
            }

            // create child, set parent and add to open map
            // heuristic = approximate heuristic using Manhattan Distance 
            // (as opposed to Euclidean Distance or Diagonal Distance)
            childNode = new Node(newX, newY, heuristic);
            childNode->setParent(parentNode, dg);
            openNodes[newNodePoint] = childNode;


            // target acquired
            if (newX == target.x && newY == target.y) {

                // generate vector of path starting from childNode till startNode
                traverseNode = childNode;
                while (traverseNode != NULL) {
                    // create Point for path form node x and y
                    path.push_back(Point(traverseNode->x, traverseNode->y));
                    traverseNode = traverseNode->parent;
                }

                //reverse vector (this is faster than inserting the points at the 
                //front, which would cause a shift every time
                reverse(path.begin(), path.end());

                // garbage collection -- clear out the open and closed nodes so we get no memory leaks (bad)
                for(auto &p : openNodes){
                    delete p.second;
                }
                openNodes.clear();

                for(auto &p : closedNodes){
                    delete p.second;
                }
                closedNodes.clear();

                return path;
            }
        }
    }

    std::cout << "never found target" << std::endl;

    // if target is never found, return an empty path :(
    return path;
}

vector<Point> MazeMapper::optimizePath(const vector<Point> &moves) {
    //this function currently can end up with targetLocations occuring twice in a single array.
    //options - ignore that and let the drve people just encounter something twice in a row
    //clear them out at the end
    //specifically ignore them as we're adding.  I choose this one.
    //diagonalize to create shortest possible path
    //grab the starting point, with line to second wayPoint.  Increment end point of that line 
    //along the path until hitting a wall, then back one and make that point the second waypoint
    //then repeat from that point.  This is not a perfect optimization.  But it's good enough for now

    vector<Point> optMoves;
    Point startPoint(getRobotPos());
    Point endPoint(moves[0]);

    //for each possible improvement
    for(unsigned i = 0; i < moves.size() - 1; ++i){
        Point nextMove(moves[i + 1]);

        //fix this to be more robust
        Point direction(nextMove - endPoint);
        direction /= abs(endPoint.x - nextMove.x + endPoint.y - nextMove.y);

        while(endPoint != nextMove){ //if we reach the next point we have a straight digaonal path to it.
            if(pathIsBlocked(startPoint, endPoint)){ //this path is not okay
                endPoint -= direction; //go back a step, we overshot
                break;
            } else {
                endPoint += direction;
            }
        }
        //if we reach the net point, we actually want to continue without pushing the waypoint.
        //how do we do that?  we want to advance the endPoint but not the oldPoint.
        //also gets rid of this big check thing
        //coo
        if(endPoint != nextMove){
            optMoves.push_back(endPoint);
            startPoint = endPoint;
        }
        endPoint = nextMove;
        //avoid including a waypoint multiple times. Trust me it could happen otherwise
    }

    //make sure we don't double count the last move which isn't part of the above loop
    if(optMoves.empty() || moves.back() != optMoves.back()){
        optMoves.push_back(moves.back());
    }

    return optMoves;
}

bool MazeMapper::pathIsBlocked(const Point &start, const Point &end){
    //creates fatter version of line (three cells wide) and iterates along that path until reaching target destination or colliding with a WALL
    //if we hit a wall, the path is blocked
    //if we make it to the end point, the path is CLEAR

    float magnitude = sqrt(pow(end.x - start.x, 2) + pow(end.y - start.y, 2));
    DoublePoint direction((DoublePoint(end) - DoublePoint(start))/magnitude);
    DoublePoint offset2(-direction.y, direction.x);
    DoublePoint offset3(direction.y, -direction.x);

    Point currentCell;
    DoublePoint currentCell2, currentCell3;//currentCell is base line, 2 and 3 add thickness to line

    for (int i = 0; i < magnitude + 1; ++i){
        //iterate along the path
        currentCell.x = static_cast<int>(start.x + static_cast<int>(direction.x * i));
        currentCell.y = static_cast<int>(start.y + static_cast<int>(direction.y * i));
        currentCell2 = currentCell + offset2;
        currentCell3 = currentCell + offset3;
        if(
                occGrid.getValue(currentCell.x,  currentCell.y) == WALL){// ||
            //occGrid.getValue(currentCell2.x, currentCell2.y) == WALL ||
            //occGrid.getValue(currentCell3.x, currentCell3.y) == WALL){
            return true;
        }
        }
        return false;
    }

    void MazeMapper::convertToDeltas(vector<Point> &moves) {
        return;//no more converting to deltas, quick fix for now.
        //moves is originally in form of absolute locations to move to, this fuction converts those to delta locations.
        //literally just returns a vector of moves[i] - moves[i-1]
        vector<Point> oldMoves(moves);
        for(unsigned int i = moves.size() - 1; i > 0; --i){
            moves[i] = oldMoves[i] - oldMoves[i-1];
        }
        moves[0] = oldMoves[0] - getRobotPos();
    }

    /////////////////////////////

    void MazeMapper::laserScanLoop() { //loops updateOccupancyGrid()
        Logger::log("starting laserScanLoop");
        Lidar lidar;
        while(lidar.getRPM(0) < 300 && !Robot::done){
            cout << lidar.getRPM(0) << endl;
            deque<int> scan = lidar.scan();
            //for(int i = 0; i < scan.size(); i++)
            //    cout << scan[i] << ", ";
            //cout << endl;
        } 
        time_sleep(5);
        cout << "RPM: " << lidar.getRPM(0) << endl; 
        while(!Robot::done){
            double prevAngle = getRobotAngle();
            deque<int> scan = lidar.scan();
            if(abs(prevAngle - getRobotAngle()) < M_PI / 10 && scan.size() != 0)
                updateOccupancyGrid(scan);
        }
            //occGrid.print(510, 710);
        Logger::log("quitting laserScanLoop");
    }

    void MazeMapper::updateOccupancyGrid(deque<int> scan){ //gets laser data and updates grid potentially have running on interrupt somehow whenever we get a laser scan
        Logger::log("updating occupancy grid");
        DoorFinder d(occGrid);
        vector<int> clean = d.average(scan, 45);
        //cout << "real scan: ";

        //for(unsigned i = 0; i < scan.size(); i++)
        //    cout << scan[i] << ", ";
        //cout << endl;

        //cout << "clean scan: ";

        //for(unsigned i = 0; i < clean.size(); i++)
        //    cout << clean[i] << ", ";
        //cout << endl;

        double newAngleSize = 360 / 45;

        //getDirection - rad
        if(getDirection != 7)
             sillySLAM(clean);


        for(unsigned i = 0; i < clean.size(); ++i){ //for each element in the scan
            int one = i - 1;
            int two = i;
            if(i == 0){
                one = clean.size() - 1;
                two = i;
            }
            //draw a line between start and end points, this is a wall   
            int startAngle = one * newAngleSize;
            int endAngle = two * newAngleSize;
            double offset = 105 * M_PI / 180;
            float x1 = clean[one] / 10.0 * cos((startAngle) * M_PI / 180 + getRobotAngle() + offset);
            float y1 = clean[one] / 10.0 * sin((startAngle) * M_PI / 180 + getRobotAngle() + offset);
            float x2 = clean[two] / 10.0 * cos((endAngle) * M_PI / 180 + getRobotAngle() + offset);
            float y2 = clean[two] / 10.0 * sin((endAngle) * M_PI / 180 + getRobotAngle() + offset);

            float dx = (x2 - x1) / newAngleSize;
            float dy = (y2 - y1) / newAngleSize;

            for(int j = 0; j < newAngleSize; j++){
                int len = sqrt(pow(x1,2) + pow(y1, 2));
                for(int k = 0; k < len; k++){ //Everything between us and the wall is clear
                    occGrid.update(getRobotPos().x + (x1 / len)*k - ((startAngle + j) / 360)*(getRobotPos().x - DoorFinder::prevPos.x), 
                            getRobotPos().y + (y1 / len)*k - ((startAngle + j) / 360)*(getRobotPos().y - DoorFinder::prevPos.y), CLEAR);
                }
                if(abs(clean[two] - clean[one]) <= 150){ //If there is no sharp difference in distance, draw a wall
                    occGrid.update((int)(getRobotPos().x + x1- ((startAngle + j) / 360)*(getRobotPos().x - DoorFinder::prevPos.x)), 
                            (int)(getRobotPos().y + y1 - ((startAngle + j) / 360)*(getRobotPos().y - DoorFinder::prevPos.y)), WALL);
                }
                x1 += dx;
                y1 += dy;
            }
        }
        //cout << "occgrid..." << endl;

        //  for(int i = 580; i < 640; i ++){
        //    for(int j = 580; j < 640; j ++){
        //     cout << occGrid.getValue(i, j);
        //   }
        // cout << endl;
        //      }

        // call find doors & hallways, which will update important values
        setTargetPoints(d.findDoorsAndHallways(scan, getTargetPoints()));

        // call find flame, which will update important values -- TODO

        // now that we have all important values for this scan, we can update the occ grid
        // i think this is how you do this for a map lol
        for(auto element : getTargetPoints()){ //element.first is the key, element.second is the value
            for(unsigned i = 0; i < element.second.size(); i++){
                occGrid.update(element.second[i].x, element.second[i].y, element.first);
            }
        }
        cout << "angle = " << getRobotAngle() << endl;
        //occGrid.print(getRobotPos().x - 100, getRobotPos().x + 100);
    }

   void MazeMapper::sillySLAM(vector<int> clean){ 
        DoublePoint start;
        DoublePoint end;
        double dir = getDirection() * 180 / M_PI;
        double dirRad = getDirection();

        int index = (dir / 360) * 45; //need to get more accurate than this, let's do some math

        int startAngle = index * 8;
        int endAngle = (index + 1) * 8;
        double offset = 105 * M_PI / 180;
        start.x = clean[index] / 10.0 * cos((startAngle) * M_PI / 180 + getRobotAngle() + offset);
        start.y = clean[index] / 10.0 * sin((startAngle) * M_PI / 180 + getRobotAngle() + offset);
        end.x = clean[(index + 1) % 45] / 10.0 * cos((endAngle) * M_PI / 180 + getRobotAngle() + offset);
        end.y = clean[(index + 1) % 45] / 10.0 * sin((endAngle) * M_PI / 180 + getRobotAngle() + offset);

        DoublePoint final = (start * (endAngle - dir) + end * (dir - startAngle)) / 8;

        int dist = sqrt(pow(final.x - getRobotPos().x, 2) + pow(final.y - getRobotPos().y));

        DoublePoint lastPos = getRobotPos();
        int oldDist;
        for(oldDist = 0; ; oldDist++){
            if(occGrid.getValue(lastPos.x + oldDist*cos(dirRad), lastPos.y + oldDist*sin(dirRad), true) != CLEAR && 
                    occGrid.getValue(lastPos.x + oldDist*cos(dirRad), lastPos.y + oldDist*sin(dirRad), true) != FAKE)
                break;
        }

        double diff = dist - oldDist;
        double thresh = 30;
        if(diff < thresh)//makes sure to not fuck everything up if we shift a little bit and look past a wall
            setRobotPos(getRobotPos() + DoublePoint(diff*cos(dirRad), diff*sin(dirRad)));        
   }
    /////////////////////////////
    Point MazeMapper::computeDistanceField() { //takes type of target, called in find

        //determine appropriate items to look for
        vector<Point> boundary;
        boundary.push_back(getRobotPos());
        vector<Point> neighbors;
        distanceField[getRobotPos().x][getRobotPos().y] = 0;
        Point currentCell;
        int currentDistance;

        while (!boundary.empty()) {
            currentCell = boundary.front();
            neighbors = findOpenNeighbors(currentCell);
            currentDistance = distanceField[currentCell.x][currentCell.y];
            for (Point neighbor : neighbors) {
                if(distanceField[neighbor.x][neighbor.y] == -1){//neighbor not already index by function
                    // if point is unknown, check to see if local area is made up of unknowns as well
                    if (occGrid.getValue(neighbor) == -1 && unknownLargeEnough(neighbor)){
                        // this point actually does represent an unkown region`
                        distanceField[neighbor.x][neighbor.y] = currentDistance + 1;
                        return neighbor;
                    }

                    distanceField[neighbor.x][neighbor.y] = currentDistance + 1;
                    boundary.push_back(neighbor);
                }
            }
            boundary.erase(boundary.begin());
        }
        return Point(-1, -1);
    }


    vector<Point> MazeMapper::findOpenNeighbors(const Point &currentPos) {
        vector<Point> openNeighbors;
        for (int x_offset = -1; x_offset < 2; ++x_offset) {
            for (int y_offset = -1; y_offset < 2; ++y_offset) {
                if (!isDiag(x_offset, y_offset) &&
                        occGrid.getValue(currentPos.x + x_offset, currentPos.y + y_offset) <= CLEAR_THRESHOLD) {
                    openNeighbors.push_back(Point(currentPos.x + x_offset, currentPos.y + y_offset));
                }
            }
        }
        return openNeighbors;
    }

    bool MazeMapper::isDiag(int x_offset, int y_offset) {
        return ((x_offset + y_offset + 2) % 2 == 0);
    }


    // returns true if area is all unknowns, false otherwise
    bool MazeMapper::unknownLargeEnough(Point center){
        int areaSize  = 2; // (2 results in an inclusive 3*3 grid)

        for(int i = center.x - (areaSize - 1); i < center.x + areaSize; i++){
            for(int j = center.y - (areaSize - 1); j < center.y + areaSize; j++){
                if(occGrid.getValue(i, j) != -1) return false; // something other than unknown has been found, so return false
            }
        }

        // all unknowns
        return true;
    }




    //TESTING FUNCTIONS ==========================================================================================================
    //honestly at this point I should just make up a whole freaking mini occGrid for the robot to work with.  Bleagh again.

    void MazeMapper::testFindNextTarget(){//prolly just make a bunch of test cases and see where it wants us to go, display in grid
        occGrid.initFakeWorld(25);

        Logger::log("Testing MazeMapper findNextTarget");

        for(unsigned int i = 0; i < distanceField.size(); i ++){
            for(unsigned int j = 0; j < distanceField[0].size(); j ++){
                distanceField[i][j] = -1;
            }
        }

        /* for(int i = 0; i <= 25; i ++){
           for(int j = 0; j <= 25; j ++){
           if(i == getRobotPos().x && j == getRobotPos().y)
           std::cout << std::setw(2) << 8;
           else
           std::cout << std::setw(2) << occGrid.getValue(i, j);
           }
           std::cout << endl;
           }*/


        GameState state;
        robotOps op;
        Point targetLoc;
        std::vector<Point> moves;

        targetPoints[FLAME].push_back(Point(23, 2));
        targetPoints[SAFE_ZONE].push_back(Point(19, 23));
        targetPoints[LEFT_SIDE_CRADLE].push_back(Point(4, 6));
        targetPoints[DOOR].push_back(Point(7, 1));
        targetPoints[DOOR].push_back(Point(12, 10));

        vector<string> opList;
        opList.push_back("OP_NOTHING");
        opList.push_back("OP_CRADLE_SIDE");
        opList.push_back("OP_CRADLE_FRONT");
        opList.push_back("OP_SAFE_ZONE");
        opList.push_back("OP_EXTINGUISH");
        opList.push_back("OP_SCANROOM");
        opList.push_back("OP_EXIT_ROOM");
        opList.push_back("OP_HALLWAY");
        opList.push_back("OP_HALLWAY_SIMPLE");
        opList.push_back("OP_STOP");

        //now just try a bunch of gamestate settings and see what goes down.

        state.levelOfCompetition = 1; //should always go to candles, ignoring baby.
        moves = findNextTarget(state, op, targetLoc);
        std::cout << state << "targetLoc: " << targetLoc.x << " " << targetLoc.y << " " << "robotOperation: " << opList[op] << std::endl << std::endl;

        state.levelOfCompetition = 3; //not in second arena, so it should go to the unknown.  Would go to hallway but DNE
        moves = findNextTarget(state, op, targetLoc);
        std::cout << state << "targetLoc: " << targetLoc.x << " " << targetLoc.y << " " << "robotOperation: " << opList[op] << std::endl << std::endl;

        state.secondArena = true; //should go to the cradle side
        moves = findNextTarget(state, op, targetLoc);
        std::cout << state << "targetLoc: " << targetLoc.x << " " << targetLoc.y << " " << "robotOperation: " << opList[op] << std::endl << std::endl;

        state.babyObtained = true; //should go to the safe zone
        moves = findNextTarget(state, op, targetLoc);
        std::cout << state << "targetLoc: " << targetLoc.x << " " << targetLoc.y << " " << "robotOperation: " << opList[op] << std::endl << std::endl;

        std::cout << "targetLoc: " << targetLoc.x << " " << targetLoc.y << " " << "robotOperation: " << op << std::endl;

        state.inRoom = true;
        moves = findNextTarget(state, op, targetLoc);
        std::cout << state << "targetLoc: " << targetLoc.x << " " << targetLoc.y << " " << "robotOperation: " << opList[op] << std::endl << std::endl;

        //print out targetLoc, op.
        //display moves in grid

        //display results from this call, then repeat with all sorts of different gamestatesi

    }

    void MazeMapper::testDetermineRobotOp(){
        //this one's just a bunch of if statements, bleagh
        //gonna leave this one be for now, it'd just be a repeat of the actual function which isn't even legit yet
    }

    void MazeMapper::testSpecialTargetPath(){
        //this pulls out Astar, dunno much aboot that.  
        //do findNextTarget first

        Logger::log("MazeMapper test: testing specialTargetPath() via findNextTarget()");


        // dummy data
        occGrid.initFakeWorld(25);

        //insert into targetPoint
        targetPoints[CANDLE].push_back(Point(3,20));

        GameState state(2,0,false,false,false,false,false);
        //MazeMapper::robotOps nextRobotOperation = MazeMapper::robotOps::OP_NOTHING;

        Point targetLocation;
        vector<Point> path;
        // draw path

        /*for(int j = 0; j < 25; j++){
          for(int i = 0; i < 25; i++){
          if(i == getRobotPos().x && j == getRobotPos().y){
          std::cout << std::setw(2) << "R";
          } else {
          std::cout << std::setw(2) << occGrid.getValue(i,j);
          }
          }
          std::cout << std::endl;
          }
          */  //really just need cases for every possible target type.  Can do.

        int targetIndex;
        Point targetLoc;

        targetPoints[FLAME].push_back(Point(23, 2));
        targetPoints[SAFE_ZONE].push_back(Point(19, 23));
        targetPoints[RIGHT_SIDE_CRADLE].push_back(Point(4, 6));
        targetPoints[DOOR].push_back(Point(7, 1));
        targetPoints[DOOR].push_back(Point(12, 10));

        path = specialTargetPath(CANDLE, targetPoints[CANDLE], targetIndex, targetLoc);

        std::cout << "Candle at (3, 20), path leads to (" << path[path.size()-1].x << ", " << path[path.size()-1].y << ") with targetLoc as (" << targetLoc.x << ", " << targetLoc.y << ")" << std::endl;
        path = specialTargetPath(FLAME, targetPoints[FLAME], targetIndex, targetLoc);
        std::cout << "Flame at (23, 2), path leads to (" << path[path.size()-1].x << ", " << path[path.size()-1].y << ") with targetLoc as (" << targetLoc.x << ", " << targetLoc.y << ")" << std::endl;
        path = specialTargetPath(RIGHT_SIDE_CRADLE, targetPoints[RIGHT_SIDE_CRADLE], targetIndex, targetLoc);
        std::cout << "red side cradle at (4, 6), path leads to (" << path[path.size()-1].x << ", " << path[path.size()-1].y << ") with targetLoc as (" << targetLoc.x << ", " << targetLoc.y << ")" << std::endl;
        path = specialTargetPath(DOOR, targetPoints[DOOR], targetIndex, targetLoc);
        std::cout << "door at (7, 1), path leads to (" << path[path.size()-1].x << ", " << path[path.size()-1].y << ") with targetLoc as (" << targetLoc.x << ", " << targetLoc.y << ")" << std::endl;

    }

    void MazeMapper::testCreateTargetPath(){
        //hoo boy.
        Logger::log("MazeMapper test: createTargetPath");
        for(unsigned int i = 0; i < distanceField.size(); i ++){
            for(unsigned int j = 0; j < distanceField[0].size(); j ++){
                distanceField[i][j] = -1;
            }
        }
        setRobotPos(DoublePoint(5, 5));
        occGrid.initFakeWorld(25);
        Point target = computeDistanceField();
        vector<Point> path = createTargetPath(target);

        for(unsigned int i = 0; i < path.size(); i ++){
            occGrid.update(path[i].x, path[i].y, i+1);
        }

        for(int i = 0; i <= 25; i ++){
            for(int j = 0; j <= 25; j ++){
                if(i == getRobotPos().x && j == getRobotPos().y)
                    std::cout << std::setw(2) << 8;
                else
                    std::cout << std::setw(2) << occGrid.getValue(i, j);
            }
            std::cout << endl;
        }
    }

    void MazeMapper::testAStar(){

    }

    void MazeMapper::testOptimizePath(){
        //gonna want a more in depth test of this one, but seems correct on this simple test
        //another visual one, takes in actual waypoints not deltas 
        Logger::log("MazeMapper test: optimizePath");
        for(unsigned int i = 0; i < distanceField.size(); i ++){
            for(unsigned int j = 0; j < distanceField[0].size(); j ++){
                distanceField[i][j] = -1;
            }
        }

        setRobotPos(DoublePoint(5, 5));
        occGrid.initFakeWorld(25);
        vector<Point> path = createTargetPath(computeDistanceField());
        vector<Point> opt = optimizePath(path);
        //best way to display a path in my mind is just to put ints in for coordinates in path
        for(unsigned int i = 0; i < opt.size(); i ++){
            occGrid.update(opt[i].x, opt[i].y, i+1);
        }

        for(int i = 0; i <= 25; i ++){
            for(int j = 0; j <= 25; j ++){
                if(i == getRobotPos().x && j == getRobotPos().y)
                    std::cout << std::setw(2) << 8;
                else
                    std::cout << std::setw(2) << occGrid.getValue(i, j);
            }
            std::cout << endl;
        }

    }

    void MazeMapper::testConvertToDeltas(){
        Logger::log("MazeMapper test: convertToDeltas");
        for(unsigned int i = 0; i < distanceField.size(); i ++){
            for(unsigned int j = 0; j < distanceField[0].size(); j ++){
                distanceField[i][j] = -1;
            }
        }
        MazeMapper mapper;
        std::vector<Point> moves;
        moves.push_back(Point(2, 3));
        moves.push_back(Point(4, 5));
        moves.push_back(Point(1, 4));
        mapper.convertToDeltas(moves);

        if(moves[0] == Point(2, 3) && moves[1] == Point(2, 2) && moves[2] == Point(-3, -1)){
            Logger::log("/tPassed");
        }
        else
            Logger::log("/tFailed", Logger::HIGH);
    }

    void MazeMapper::testPathIsBlocked(){
        Logger::log("MazeMapper test: pathIsBlocked");
        for(unsigned int i = 0; i < distanceField.size(); i ++){
            for(unsigned int j = 0; j < distanceField[0].size(); j ++){
                distanceField[i][j] = -1;
            }
        }
        occGrid.initFakeWorld(25);
        setRobotPos(DoublePoint(12, 12));
        Logger::log("occupancy Grid: ");
        for(int i = 0; i <= 25; i ++){
            for(int j = 0; j <= 25; j ++){
                if(i == getRobotPos().x && j == getRobotPos().y)
                    std::cout << std::setw(2) << 8;
                else
                    std::cout << std::setw(2) << occGrid.getValue(i, j);
            }
            std::cout << endl;
        }
        Logger::log("Clear Paths:");
        for(int i = 0; i <= 25; i ++){
            for(int j = 0; j <= 25; j++){
                Point end(i, j);
                if(pathIsBlocked(getRobotPos(), end))
                    std::cout << std::setw(2) << 1;
                else
                    std::cout << std::setw(2) << 0;
            }
            std::cout << std::endl;
        }
    }

    void MazeMapper::testIsDiag(){
        Logger::log("MazeMapper test: isDiag");
        for(unsigned int i = 0; i < distanceField.size(); i ++){
            for(unsigned int j = 0; j < distanceField[0].size(); j ++){
                distanceField[i][j] = -1;
            }
        }
        if(isDiag(1, 0) || isDiag(0, 1) || isDiag(-1, 0) || isDiag(0, -1))
            Logger::log("/tFailed: false positive", Logger::HIGH);
        if(!isDiag(1, 1) || !isDiag(1, -1) || !isDiag(-1, 1) || !isDiag(-1, -1))
            Logger::log("/tFailed: false negative", Logger::HIGH);
    }

    void MazeMapper::testUpdateOccupancyGrid(){//this one'll be demon in and of itself.  probably compartmentalize for less hell

    }

    void MazeMapper::testComputeDistanceField(){
        //can run on small subsection of a map to see well enough.
        //this should probs be one of those "look and see if it's right" kinda gigs
        Logger::log("MazeMapper test: computeDistanceField");
        //dang, need direct access to occupancyGrid again, don't I.
        for(unsigned int i = 0; i < distanceField.size(); i ++){
            for(unsigned int j = 0; j < distanceField[0].size(); j ++){
                distanceField[i][j] = -1;
            }
        }
        setRobotPos(DoublePoint(5, 5));
        occGrid.initFakeWorld(25);
        Point target = computeDistanceField();
        Logger::log("distanceField: ");
        for(int i = 0; i <= 25; i ++){
            for(int j = 0; j <= 25; j ++){
                if(i == target.x && j == target.y)
                    std::cout << std::setw(2) << '*';
                else
                    std::cout << std::setw(2) << distanceField[i][j];
            }
            std::cout << std::endl;
        }

        std::cout << std::endl;
        Logger::log("occupancy Grid: ");
        for(int i = 0; i <= 25; i ++){
            for(int j = 0; j <= 25; j ++){
                std::cout << std::setw(2) << occGrid.getValue(i, j);
            }
            std::cout << endl;
        }

    }

    void MazeMapper::testComputePathLength(){
        Logger::log("MazeMapper test:  computePathLength");
        vector<Point> deltas;
        deltas.push_back(Point(3, 4));
        deltas.push_back(Point(2, 3));
        deltas.push_back(Point(-3, 4));
        deltas.push_back(Point(0, -10));
        deltas.push_back(Point(3, -5));
        float length = computePathLength(deltas);
        if(abs(length - 29.4365) < .01){
            Logger::log("\tPassed");
            //return true;
            return;
        }
        Logger::log("\tFailed", Logger::HIGH);
        //return false;
    }
