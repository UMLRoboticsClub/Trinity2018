#include "globals.h"
#include "constants.h"
#include "Point.h"
#include "Node.h"
#include "OccupancyGrid.h"

#include <vector>
#include <unordered_map>
#include <cmath>
#include <algorithm>
#include <deque>

using namespace std;

class DoorFinder{
public:
    DoorFinder();
    void findDoorsAndHallways(LaserScanner scan, map<int, vector<Point>>& targetPoints, OccupancyGrid occGrid);
private:
    //add more private variables as needed TODO
    double angleSize;

    void updateAngleSize();
    std::vector<DoublePoint> averagePoints(std::vector<DoublePoint> points);
    double findDistance(vector<double> averaged, int peak);
    vector<double> getSubsection(deque<double> scan, int start, int end);
    bool isPeakHallway(vector<double> subsection);
    vector<double> average(deque<double> scan, int numSections);
	vector<int> findPeaks(vector<double> averaged, int start, int nd, bool checkEndpoints);
	vector<int> sortPeaks(deque<double> scan, vector<double> averaged, vector<int> peaks, bool isHallway);
	vector<DoublePoint> pointsFromPeaks(vector<double> averaged, vector<int> peaks, int rotation);
};

