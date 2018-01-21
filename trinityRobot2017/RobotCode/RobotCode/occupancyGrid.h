/* 
 * File:   occupancyGrid.h
 * Author: Mong Mary Touch
 *
 * Created on November 7, 2017, 6:02 PM
 */


#ifndef OCCUPANCYGRID_H
#define	OCCUPANCYGRID_H
#include "NavVal.h"
#include "Point.h"
#include <vector>

using namespace std;

class OccupancyGrid {
public:
    OccupancyGrid();
	OccupancyGrid(const OccupancyGrid& orig);
    // constructor initialize to width and height 
    
    int update(float realX, float realY, int val);
	int getHeight();
	int getWidth();
    int getValue(int x, int y);
    int getValue(Point point);   // return array

	int getTotalWidth(void);
	int getTotalHeight(void);
    
private:
    vector<vector<NavVal>> gridVals;
    int width;
    int height;
    int resolution;
};

#endif	/* OCCUPANCYGRID_H */