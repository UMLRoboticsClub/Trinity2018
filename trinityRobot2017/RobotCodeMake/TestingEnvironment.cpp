#include "TestingEnvironment.h"

//testing functions for gamestate
//set various flags and make sure gamestate returns the proper priority by state
bool TestingEnvironment::testGameState(){
  GameState state;
  state.levelOfCompetition = 1;
  state.babySaved = false;
  state.babyObtained = false;
  state.safeZoneFound = false;
  state.inRoom = false;
  state.secondArena = false;
  vector<int> targets = state.getTargetType();
  if(targets[0] != FLAME || targets[1] != CANDLE || targets[2] != DOOR)
    return false;
  state.levelOfCompetition = 2;
  if(targets[0] != FLAME || targets[1] != CANDLE || targets[2] != DOOR)
    return false;
  state.levelOfCompetition = 3;
  //just beginning level three, prioritize finding the hallway
  if(targets[0] != HALLWAY)
    return false;
  state.secondArena = true;
  //we are in the second arena
  //first thing we do would be to immediately locate the safeZone
  state.safeZoneFound = true;
  if(targets[0] != RED_SIDE_CRADLE || targets[1] != BLUE_SIDE_CRADLE || targets[2] != GREEN_SIDE_CRADLE || targets[3] != DOOR)
    return false;
  state.babyObtained = true;
  if(targets[0] != SAFE_ZONE)
    return false;
  state.babySaved = true;
  //once the baby is saved, we prioritize candles and shit
  if(targets[0] != FLAME || targets[1] != CANDLE || targets[2] != DOOR)
    return false;
  state.babySaved = false;
  state.safeZoneFound = false;
  if(targets[0] != SAFE_ZONE)
    return false;

  return true;
}

//mazemapper tests oh boy.
//might want to seperate functions further
//for better compartmentalization of tests.
bool TestingEnvironment::testComputePathLength(){
  vector<Point> deltas;
  deltas.push_back(Point(2, 2));
  deltas.push_back(Point(3, -5));
  deltas.push_back(Point(-2, 0));
  MazeMapper mapper;
  //prolly did the math for this right
  return abs(mapper.computePathLength(deltas) - 10.6594) < .1;
}
bool TestingEnvironment::testFindNextTarget(){
  //lotsa testing for this dude.  Might want to compartmentalize it for easier testing

    return false;
}
bool TestingEnvironment::testCreateTargetPath(){
    return false;
}
bool TestingEnvironment::testAStar(){
  //make Sam write this one
    return false;
}
bool TestingEnvironment::testOptimizePath(){
  //not sure what the best way to test this one is
    return false;
}
bool TestingEnvironment::testConvertToDeltas(){
  MazeMapper mapper;
  //robotPos = Point(0, 0);
  std::vector<Point> ogMoves;
  ogMoves.push_back(Point(2, 3));
  ogMoves.push_back(Point(4, 5));
  ogMoves.push_back(Point(1, 4));
  std::vector<Point> deltaMoves = mapper.convertToDeltas(ogMoves);
  return (deltaMoves[0] == Point(2, 3) && deltaMoves[1] == Point(2, 2) && deltaMoves[2] == Point(-3, -1));
}
bool TestingEnvironment::testPathIsBlocked(){
    return false;
}
bool TestingEnvironment::testIsDiag(){
    return false;
}
bool TestingEnvironment::testUpdateOccupancyGrid(){
    return false;
}

//honestly best way to test those would be to legit test them.
bool TestingEnvironment::testComputeDistanceField(){
    return false;
}
bool TestingEnvironment::testFindOpenNeighbors(){
    return false;
}

//occupancyGridTests
//just test our getters and setters and stuff
bool testOccGrid(){
    return false;
}
