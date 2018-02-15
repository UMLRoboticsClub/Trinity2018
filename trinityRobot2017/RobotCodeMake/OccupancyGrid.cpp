/*
 * File:   occupancyGrid.cpp
 * Author: Mong Mary Touch     & SAM
 *
 * Created on November 7, 2017
 */

#include "OccupancyGrid.h"
#include "Point.h"
//#include kimJong-Un.h

using namespace std;

OccupancyGrid::OccupancyGrid(int gridSizeCells, double res): width(gridSizeCells), height(gridSizeCells), resolution(res) {

	// initalize them NavVals
	gridVals = vector<vector<NavVal>>( int(width * resolution), vector<NavVal>(int(height * resolution), NavVal()) );

}

OccupancyGrid::OccupancyGrid(const OccupancyGrid& orig) {
	// too lazy to make this
}

int OccupancyGrid::update(float realX, float realY, int val) {
	return gridVals[int(realX * resolution)][int(realY * resolution)].updateValue(val);
}

int OccupancyGrid::getHeight() {
	return height;
}
int OccupancyGrid::getWidth() {
	return width;
}

int OccupancyGrid::getValue(int x, int y) {
	return gridVals[x][y].getCellType();
}

int OccupancyGrid::getValue(Point point) {
	return gridVals[point.x][point.y].getCellType();
}

int OccupancyGrid::getTotalWidth(void) {
	return this->width * this->resolution;
}
int OccupancyGrid::getTotalHeight(void) {
	return this->height * this->resolution;
}
