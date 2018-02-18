/*
 * Author: Mong Mary Touch     & SAM
 */

#include "occupancygrid.h"
#include "point.h"

using namespace std;

OccupancyGrid::OccupancyGrid(int gridSizeCells, double res):
    width(gridSizeCells), height(gridSizeCells), resolution(res)
{

    // initalize them NavVals
    gridVals = vector<vector<NavVal>>( int(width * resolution), vector<NavVal>(int(height * resolution), NavVal()) );

}

OccupancyGrid::OccupancyGrid(const OccupancyGrid& orig) {
    width = orig.width;
    height = orig.height;
    gridVals = orig.gridVals;
    resolution = orig.resolution;
}

int OccupancyGrid::update(float realX, float realY, int val) {
    return gridVals[int(realX * resolution)][int(realY * resolution)].updateValue(val);
}

int OccupancyGrid::getValue(int x, int y) {
    return gridVals[x][y].getCellType();
}

int OccupancyGrid::getValue(Point point) {
    return gridVals[point.x][point.y].getCellType();
}

int OccupancyGrid::getTotalWidth() {
    return width * resolution;
}

int OccupancyGrid::getTotalHeight() {
    return height * resolution;
}
