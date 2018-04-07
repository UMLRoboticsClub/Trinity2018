#include "occupancygrid.h"
#include "logger.h"
#include <iostream>
#include "point.h"
#include "constants.h"
using std::cout;
using std::endl;


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
    setRobotPos(DoublePoint(4, 3));
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
    std::lock_guard<std::mutex> lock(occGridMutex);
    if(val == WALL){
        for(int i = realX - ROBOT_DIAMETER_CM / 2; i < realX + ROBOT_DIAMETER_CM / 2; i++){
            for(int j = realY - ROBOT_DIAMETER_CM / 2; j < realY + ROBOT_DIAMETER_CM / 2; j++){
                if(i > 0 && j > 0)
                    gridVals[i * RESOLUTION][j * RESOLUTION].updateValue(FAKE);
            }
        }
    }
    return gridVals[realX * RESOLUTION][realY * RESOLUTION].updateValue(val);
}

int OccupancyGrid::getValue(int x, int y, bool fakeWall) const {
    std::lock_guard<std::mutex> lock(occGridMutex);
    if(!fakeWall && gridVals[x][y].getCellType() == FAKE)
        return WALL;
    if(x < 0 || x >= size || y < 0 || y >= size)
        return 1;
    else
        return gridVals[x][y].getCellType();
}

int OccupancyGrid::getValue(const Point &point) const {
    return getValue(point.x, point.y);


}

void OccupancyGrid::print(int from, int to, int targetx = -1, int targety = -1){
    for(int i = from; i < to; i++){
        for(int j = from; j < to; j++){
            if(i == getRobotPos().x && j == getRobotPos().y)
                cout << RED << "X " << RST;
            else if(i == targetx && j == targety)
                cout << BLU << "T " << RST;
            else if(gridVals[i][j].getCellType() == WALL)
                cout << GRN << "1 " << RST;
            else if(gridVals[i][j].getCellType() == FAKE)
                cout << GRN << "F " << RST;
            else if(gridVals[i][j].getCellType() == CLEAR)
                cout << "0 ";
            else if(gridVals[i][j].getCellType() == DOOR || gridVals[i][j].getCellType() == HALLWAY)
                cout << YEL << gridVals[i][j].getCellType() << " " << RST; 
            else if(gridVals[i][j].getCellType() == UNKNOWN)
                cout << "  ";
            else
                cout << BLU << gridVals[i][j].getCellType() << " " << RST;
        }
        cout << endl;
    }
}
