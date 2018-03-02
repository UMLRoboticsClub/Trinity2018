
#include "robot.h"

int main() {

	//Robot sam;
	//sam.start();
    MazeMapper mapper;
    mapper.testCreateTargetPath();
    mapper.testOptimizePath();
    mapper.testPathIsBlocked();
    mapper.testIsDiag();
    mapper.testComputePathLength();
    mapper.testComputeDistanceField();
    return 0;
}
