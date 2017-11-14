/* 
 * File:   occupancyGrid.h
 * Author: Mong Mary Touch
 *
 * Created on November 7, 2017, 6:02 PM
 */

#ifndef OCCUPANCYGRID_H
#define	OCCUPANCYGRID_H

struct Point{
    int x;
    int y;
};

class occupancyGrid {
public:
    occupancyGrid();
    // constructor initialize to width and height 
    
    int update(float realX, float realY, int val);
    int getValue(int x, int y);
    int getValue(Point);   // return array
    
private:
    NavVal gridVals[][];
    int width;
    int height;
    int resolution;
};

#endif	/* OCCUPANCYGRID_H */

