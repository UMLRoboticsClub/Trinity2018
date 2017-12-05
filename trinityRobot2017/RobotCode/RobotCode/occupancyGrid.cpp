/* 
 * File:   occupancyGrid.cpp
 * Author: Mong Mary Touch     & SAM
 * 
 * Created on November 7, 2017
 */

#include "occupancyGrid.h"
#include "Point.h"
//#include kimJong-Un.h

using namespace std;

OccupancyGrid::OccupancyGrid(): width(500), height(500), resolution(1) {

	// initalize them NavVals
	gridVals( int(width * resolution), vector<NavVal>(int(height * resolution), NavVal()) );

}

OccupancyGrid::OccupancyGrid(const OccupancyGrid& orig) {
	// too lazy to make this
}

int OccupancyGrid::update(float realX, float realY, int val) {
	gridVals[int(realX * resolution)][int(realY * resolution)].updateValue(val);
}

int OccupancyGrid::getValue(int x, int y) {
	return gridVals[x][y];
}

int OccupancyGrid::getValue(Point point) {
	return gridVals[point.x, point.y];
}