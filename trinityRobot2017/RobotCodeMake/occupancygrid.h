/*
 * Author: Mong Mary Touch
 *
 */

#ifndef OCCUPANCYGRID_H
#define	OCCUPANCYGRID_H

#include "navval.h"
#include "point.h"
#include <vector>

using std::vector;

class OccupancyGrid {
    public:
        OccupancyGrid(int gridSize, double res);
        OccupancyGrid(const OccupancyGrid& orig);

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
