#include "occupancygrid.h"

#include "point.h"
#include "constants.h"

OccupancyGrid::OccupancyGrid():
    size(GRID_SIZE_CELLS * RESOLUTION),
    gridVals(size, vector<NavVal>(size)){}

OccupancyGrid::OccupancyGrid(const OccupancyGrid& orig): 
    size(orig.size),
    gridVals(orig.gridVals){}

int OccupancyGrid::update(float realX, float realY, int val) {
    return gridVals[realX * RESOLUTION][realY * RESOLUTION].updateValue(val);
}

int OccupancyGrid::getValue(int x, int y) const {
    return gridVals[x][y].getCellType();
}

int OccupancyGrid::getValue(const Point &point) const {
    return gridVals[point.x][point.y].getCellType();
}
