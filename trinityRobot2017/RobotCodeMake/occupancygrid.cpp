#include "occupancygrid.h"
#include <iostream>
#include "point.h"
#include "constants.h"

OccupancyGrid::OccupancyGrid():
    size(GRID_SIZE_CELLS * RESOLUTION),
    gridVals(size, vector<NavVal>(size)){
        std::cout << "doin a thing" << std::endl;
        std::cout << (gridVals[0][0].getCellType()) << std::endl; 

    }
    

OccupancyGrid::OccupancyGrid(const OccupancyGrid& orig): 
    size(orig.size),
    gridVals(orig.gridVals){}

int OccupancyGrid::update(float realX, float realY, int val) {
    return gridVals[realX * RESOLUTION][realY * RESOLUTION].updateValue(val);
}

int OccupancyGrid::getValue(int x, int y) const {
    if(x < 0 || x >= size || y < 0 || y >= size)
        return 1;
    else
        return gridVals[x][y].getCellType();
}

int OccupancyGrid::getValue(const Point &point) const {
    return getValue(point.x, point.y);
}
