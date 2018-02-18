#include "mazemapper.h"
#include "occupancygrid.h"
#include "gamestate.h"

class TestingEnvironment{
  //this class will simulate real data and test all of  our functions to see if they work as intended
  //gonna have to create a fake occupancy grid and shit

public:
  TestingEnvironment(){}
//testing functions by class

//testing functions for gamestate
bool testGameState(); //set various flags and make sure gamestate returns the proper priority by state

//mazemapper tests oh boy.
//might want to seperate functions further
//for better compartmentalization of tests.
bool testComputePathLength();
bool testFindNextTarget();
bool testCreateTargetPath();
bool testAStar();
bool testOptimizePath();
bool testConvertToDeltas();
bool testPathIsBlocked();
bool testIsDiag();
bool testUpdateOccupancyGrid();
bool testComputeDistanceField();
bool testFindOpenNeighbors();

//occupancyGridTests
bool testOccGrid();//just test our getters and setters and stuff
private:
};
