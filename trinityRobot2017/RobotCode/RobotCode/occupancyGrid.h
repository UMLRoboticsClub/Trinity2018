/* 
 * File:   occupancyGrid.h
 * Author: Mong Mary Touch
 *
 * Created on November 7, 2017, 6:02 PM
 */


#ifndef OCCUPANCYGRID_H
#define	OCCUPANCYGRID_H
#include "NavVal.h"
#include <vector>
class OccupancyGrid {
public:
    OccupancyGrid();
    // constructor initialize to width and height 
    
    int update(float realX, float realY, int val);
    int getValue(int x, int y);
    int getValue(Point);   // return array
    
private:
    vector<vector<NavVal>> gridVals;
    int width;
    int height;
    int resolution;
};

#endif	/* OCCUPANCYGRID_H */

