// constants for NavVal
#ifndef CONSTANTS_H
#define CONSTANTS_H

const float static CLEAR_THRESHOLD = 0.25;

const double RESOLUTION = 1;

constexpr int ARENALENGTH_CM = 244;
constexpr int CELLSIZE_CM = 1;
constexpr int ROBOT_DIAMETER_CM = 31;
//big enough to hold entire maze no matter where we start
const int GRID_SIZE_CM  = 5 * ARENALENGTH_CM;
const int GRID_SIZE_CELLS = GRID_SIZE_CM / RESOLUTION;
// Constant cell types, used as parameters to updateValue().
enum cellType { 
    UNKNOWN = -1,
    CLEAR,
    WALL,
    DOOR,
    EXPLORED_DOOR,
    HALLWAY,
    FLAME,
    CANDLE,
    EXTINGUISHED,
    RED_SIDE_CRADLE,
    BLUE_SIDE_CRADLE,
    GREEN_SIDE_CRADLE,
    SAFE_ZONE,
    START_ZONE
};

#endif
