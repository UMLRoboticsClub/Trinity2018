// constants for NavVal

#pragma once

static const float CLEAR_THRESHOLD = 0.25;
static const int RESOLUTION = 1;

constexpr static const int ARENA_LENGTH_CM = 244;
constexpr static const int HALLWAY_WIDTH_CM = 46;
constexpr static const int CELLSIZE_CM = 1;
constexpr static const int ROBOT_DIAMETER_CM = 31;

constexpr static const int CRADLE_SIZE_CM = 15;

//big enough to hold entire maze no matter where we start
static const int GRID_SIZE_CM  = 5 * ARENA_LENGTH_CM;
static const int GRID_SIZE_CELLS = GRID_SIZE_CM / RESOLUTION;

// Constant cell types, used as parameters to updateValue().
enum cellType { 
    UNKNOWN = -1,
    CLEAR,
    WALL,
    FLAME,
    CANDLE,
    EXTINGUISHED,
    SAFE_ZONE,
    LEFT_SIDE_CRADLE,
    RIGHT_SIDE_CRADLE,
    FRONT_SIDE_CRADLE,
    DOOR,
    EXPLORED_DOOR,
    HALLWAY,
    START_ZONE,
    FAKE
};

