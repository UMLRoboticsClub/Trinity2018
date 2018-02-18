/*
 * Author: Mong Mary Touch
 *
 */

#ifndef OCCUPANCYGRID_H
#define	OCCUPANCYGRID_H

#include "navval.h"
#include "point.h"
#include <vector>

using namespace std;

class OccupancyGrid {
    public:
        OccupancyGrid(int gridSize, double res);
        OccupancyGrid(const OccupancyGrid& orig);
        // constructor initialize to width and height

        int update(float realX, float realY, int val);
        int getValue(int x, int y);
        int getValue(Point point);   // return array

        int getTotalWidth();
        int getTotalHeight();

        int width, height;
    private:
        vector<vector<NavVal>> gridVals;
        double resolution;
};

#endif
