// constants for NavVal
#ifndef CONSTANTS_H
#define CONSTANTS_H

const float static CLEAR_THRESHOLD = 0.25;

// Constant cell types, used as parameters to updateValue().
// enum can't be floats so...
const float static UNKNOWN = -1;
const float static CLEAR = 0;
const float static WALL = 1;
const float static FLAME = 2;
const float static CANDLE = 3;
const float static EXTINGUISHED = 4;
const float static SAFE_ZONE = 5;
const float static RED_SIDE_CANDLE = 6;
const float static BLUE_SIDE_CANDLE = 7;
const float static GREEN_SIDE_CANDLE = 8;
const float static DOOR = 9;
const float static EXPLORED_DOOR = 10;

#endif 