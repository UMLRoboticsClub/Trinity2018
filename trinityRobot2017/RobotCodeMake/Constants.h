// constants for NavVal
#ifndef CONSTANTS_H
#define CONSTANTS_H

const float static CLEAR_THRESHOLD = 0.25;
//make this bigger than 1 so we have more than one cell
const int GRID_SIZE = 1;

// Constant cell types, used as parameters to updateValue().
// enum can't be floats so...
//I see no reason for these to be floats
const float static UNKNOWN = -1;
const float static CLEAR = 0;
const float static WALL = 1;
const float static FLAME = 2;
const float static CANDLE = 3;
const float static EXTINGUISHED = 4;
const float static SAFE_ZONE = 5;
const float static RED_SIDE_CRADLE = 6;
const float static BLUE_SIDE_CRADLE = 7;
const float static GREEN_SIDE_CRADLE = 8;
const float static DOOR = 9;
const float static EXPLORED_DOOR = 10;
const float static HALLWAY = 11;
const float START_ZONE = 12;

#endif
