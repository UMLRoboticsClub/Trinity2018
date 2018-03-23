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

void OccupancyGrid::init(){
    gridVals = std::vector<std::vector<NavVal>>(size, vector<NavVal>(size));
}

void OccupancyGrid::initFakeWorld(int fakeSize){
    this->size = fakeSize;
    init();
    //let's make a pretty little universe.
    //start robot off more or less in the top corner
    robotPos.x = 4;
    robotPos.y = 3;
    //I AM THE GOD OF THIS NEW WORLD
    for(int i = 0; i < fakeSize; i ++){
        for(int j = 0; j < fakeSize; j ++){
            if(i == fakeSize-1 || j == fakeSize-1)//surrounding walls so robot can't escape
                update(i, j, 1);
            else if(i == 6 && j < 8)
                update(i, j, 1);
            else if(j == 7 && i > 1 && i < 6)
                update(i, j, 1);
            else if (i < 9 && j == 12)
                update(i, j, 1);
            else if (i == 12 && j > 12)
                update(i, j, 1);
            else if (i == 17 && j < 20)
                update(i, j, 1);
            else if (j == 5 && i > 20)
                update(i, j, 1);
            else if (j == 12 && i > 17 && i < 22)
                update(i, j, 1);
            else if (i == 20 && j == 3)
                update(i, j, -1);
            else if (i == 23 && j == 2)
                update(i, j, FLAME);
            else if (i == 3 && j == 20)
                update(i, j, CANDLE);
            else if (i == 19 && j == 23)
                update(i, j, SAFE_ZONE);
            else
                update(i, j, 0); //default to clear
        }
    }
}

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
