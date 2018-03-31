#include "DoorFinder.h"
#include "globals.h"
#include "constants.h"
#include "Point.h"
#include "Node.h"

#include <vector>
#include <unordered_map>
#include <cmath>
#include <algorithm>
#include <deque>

//using namespace std;

DoorFinder::DoorFinder(){
    //set angle size and other stuff
    numIndex = 0;
    angleSize = 360 / NUM_SECTIONS[numIndex];
}

void DoorFinder::findDoorsAndHallways(std::deque<int> scan, map<int, vector<Point>>& targetPoints){
    std::vector<DH> allDoors;
    std::vector<DH> allHall;

    numIndex = 0;

    scan = testScan;

    //loop through x times
    for(int i = 0; i < 360; i++){
        //average current scan into sections
        std::vector<int> sections = average(scan, NUM_SECTIONS[numIndex]);

        //for(int i = 0; i < sections.size(); i++)
          //  cout << sections[i] << ", ";

        //cout << endl;

        std::vector<int> peaks = findPeaks(sections, 1, sections.size() - 1, true);
        std::vector<int> hallwayPeaks = sortPeaks(scan, sections, peaks, true);
        std::vector<int> doorPeaks = sortPeaks(scan, sections, peaks, false);
        //if(hallwayPeaks.size() != 0 || doorPeaks.size() != 0)
          //  cout << hallwayPeaks.size() << " / " << doorPeaks.size() << endl;

        if(peaks.size() == 0 && numIndex < NUM_SECTIONS.size() - 1){
            numIndex++;
            i--;
        } else {
            for(int j = 0; j < hallwayPeaks.size(); j++){
                DH hallway;
                hallway.angle = getAngle(hallwayPeaks[j], i);
                hallway.dist = findDistance(sections, hallwayPeaks[j], true);
                hallway.point =
                //std::vector<DoublePoint> hallPoints = pointsFromPeaks(sections, hallwayPeaks, i, true);
                //allHall.insert(allHall.end(), hallPoints.begin(), hallPoints.end());
            }
            for(int j = 0; j < doorPeaks.size(); j++){
                std::vector<DoublePoint> doorPoints = pointsFromPeaks(sections, doorPeaks, i, false);
                allDoors.insert(allDoors.end(), doorPoints.begin(), doorPoints.end());
            }

            double r = scan.back();
            scan.pop_back();
            scan.push_front(r);
            numIndex = 0;
        }
    }

    cout << allDoors.size() << " ||| " << allHall.size() << endl;
    //average all estimations to find out which are of the same door/hallway
    allDoors = averagePoints(allDoors);
    allHall = averagePoints(allHall);

    //for(int i = 0; i < allDoors.size(); i++)
      //  cout << robotPos.x + allDoors[i].x / 10 << " " << robotPos.y + allDoors[i].y / 10 << endl;

    for(int i = 0; i < allDoors.size(); i++)
        occGrid.update(robotPos.x + allDoors[i].x / 10, robotPos.y + allDoors[i].y / 10, DOOR);

    for(int i = 0; i < allHall.size(); i++)
        occGrid.update(robotPos.x + allHall[i].x / 10, robotPos.y + allHall[i].y / 10, HALLWAY);

    occGrid.printGrid(robotPos.x - 150, robotPos.x + 50);
    //put final results into the important values list
}

int DoorFinder::getAngle(int peak, int rotation){
    int angle = (peak + 1) * angleSize - angleSize * (3.0/4.0) - (rotation);
    angle -= (int) (robotAngle * 180 / M_PI);
    if(angle > 360)
        angle = angle % 360;
    if(angle < 0)
        angle = 360 + angle;
    return angle;
}

void DoorFinder::updateOccupancyGrid(){ //gets laser data and updates grid potentially have running on interrupt somehow whenever we get a laser scan
    vector<int> clean = average(testScan, 45);

    for(int i = 0; i < clean.size(); i++)
        cout << clean[i] << ", ";
    cout << endl;

    double newAngleSize = 360 / 45;

    robotPos.x = occGrid.size / 2;
    robotPos.y = occGrid.size / 2;
    robotAngle = 0;

    //cout << robotPos.x << " " << robotPos.y << endl;

    for(int i = 0; i < clean.size(); i++){ //for each element in the scan
        int one = i - 1;
        int two = i;
        if(i == 0){
            one = clean.size() - 1;
            two = i;
        }
        int startAngle = one * newAngleSize;
        int endAngle = two * newAngleSize;
        int angle = startAngle;
        float x1 = clean[one] / 10.0 * cos((startAngle + robotAngle) * M_PI / 180);
        float y1 = clean[one] / 10.0 * sin((startAngle + robotAngle) * M_PI / 180);
        float x2 = clean[two] / 10.0 * cos((endAngle + robotAngle) * M_PI / 180);
        float y2 = clean[two] / 10.0 * sin((endAngle + robotAngle) * M_PI / 180);

        float dx = (x2 - x1) / newAngleSize;
        float dy = (y2 - y1) / newAngleSize;

        for(int j = 0; j < newAngleSize; j++){
            int len = sqrt(pow(x1,2) + pow(y1, 2));
            for(int k = 0; k < len; k++){
                occGrid.update(robotPos.x + (x1 / len)*k, robotPos.y + (y1 / len)*k, CLEAR);
            }
            if(abs(clean[two] - clean[one]) <= 150){
                occGrid.update((int)(robotPos.x + x1), (int)(robotPos.y + y1), WALL);
            }
            x1 += dx;
            y1 += dy;
        }
    }
    //occGrid.printGrid(robotPos.x - 100, robotPos.x + 100);
}


vector<int> DoorFinder::average(deque<int> scan, int numSections){
    vector<int> averaged(numSections);
    if(scan.size() != 0){
        int numItems = scan.size() / numSections;
        int start = 0;
        int nd = numItems;
        int real = numItems;

        for(int i = 0; i < numSections; i++){
            for(int j = start; j < nd; j++){
                if(scan[j] > 60)
                    averaged.at(i) += scan[j];
                else
                    real--;
            }
            if(real == 0 && i != 0)
                averaged.at(i) = averaged.at(i - 1);
            else if(real == 0)
                averaged.at(i) = -1;
            else
                averaged.at(i) /= real;
            start = nd;
            nd += numItems;
            real = numItems;
        }
    } else {
        //ooh an error
    }
    if(averaged[0] == -1)
        averaged[0] = averaged[averaged.size() - 1];
    return averaged;
}

vector<int> DoorFinder::findPeaks(vector<int> averaged, int start, int nd, bool checkEndpoints){
    float margin = 200; //peak must be this far above its neighbors

    vector<int> indices;
    for(int i = start; i < nd; i++){
        if(averaged[i-1] + 100 <= averaged[i] || averaged[i+1] + 100  <= averaged[i]){
            //cout << averaged[i-1] - averaged[i] << " " << averaged[i+1] - averaged[i] << ", ";
        }
        if(averaged[i-1] + margin <= averaged[i] && averaged[i+1] + margin <= averaged[i]){
            indices.push_back(i);
        }
    }

    //cout << endl;

    if(checkEndpoints){
        if(averaged[start] + margin <= averaged[start - 1] && averaged[nd] + margin <= averaged[start - 1])
            indices.push_back(start - 1);
        if(averaged[nd - 1] + margin <= averaged[nd] && averaged[start - 1] + margin <= averaged[nd])
            indices.push_back(nd);
    }

    return indices;
}

vector<DoublePoint> DoorFinder::pointsFromPeaks(vector<int> averaged, vector<int> peaks, int rotation, bool isHallway){
    vector<DoublePoint> points;
    vector<int> distances;
    angleSize = 360 / NUM_SECTIONS[numIndex];

    for(int i = 0; i < peaks.size(); i++){
        distances.push_back(this->findDistance(averaged, peaks[i], isHallway));
        DoublePoint point;

        int angle = (peaks[i] + 1) * angleSize - angleSize * (3.0/4.0) - (rotation);
        angle -= (int) (robotAngle * 180 / M_PI);
        if(angle > 360)
            angle = angle % 360;
        if(angle < 0)
            angle = 360 + angle;

        //cout << NUM_SECTIONS[numIndex] << " " << angleSize << " " << rotation << " " << angle << " " << averaged[peaks[i]] << endl;

        point.x = distances[i] * cos((double) angle * M_PI / 180);
        point.y = distances[i] * sin((double) angle * M_PI / 180);

        points.push_back(point);
    }

    return points;
}

int DoorFinder::findDistance(vector<int> averaged, int peak, bool isHallway){
    vector<int> doorDist;

    if(peak == 0) {
        doorDist.push_back(averaged[averaged.size() - 1]);
        if(!isHallway)
            doorDist.push_back(averaged[averaged.size() - 2]);
    } else if(peak == 1) {
        if(!isHallway)
            doorDist.push_back(averaged[averaged.size() - 1]);
        doorDist.push_back(averaged[0]);
    } else {
        if(!isHallway)
            doorDist.push_back(averaged[peak - 2]);
        doorDist.push_back(averaged[peak - 1]);
    }

    if(peak == averaged.size() - 1){
        doorDist.push_back(averaged[0]);
        if(!isHallway)
            doorDist.push_back(averaged[1]);
    } else if(peak == averaged.size() - 2){
        if(!isHallway)
            doorDist.push_back(averaged[0]);
        doorDist.push_back(averaged[averaged.size() - 1]);
    } else {
        doorDist.push_back(averaged[peak + 1]);
        if(!isHallway)
            doorDist.push_back(averaged[peak + 2]);
    }

    return (doorDist[0] + doorDist[1]) / 2;
}

vector<int> DoorFinder::sortPeaks(deque<int> scan, vector<int> averaged, vector<int> peaks, bool isHallway){
    vector<int> sortedPeaks;
    int numSections = averaged.size();
    float sectionWidth = scan.size() / numSections;

    for(int i = 0; i < peaks.size(); i++){
        vector<int> subsection = this->getSubsection(scan, (peaks[i] * sectionWidth - 20), (peaks[i] * sectionWidth + 20));
        if(this->isPeakHallway(subsection) && isHallway)
            sortedPeaks.push_back(peaks[i]);
        else if(!this->isPeakHallway(subsection) && !isHallway)
            sortedPeaks.push_back(peaks[i]);
    }
    return sortedPeaks;
}

vector<int> DoorFinder::getSubsection(deque<int> scan, int start, int end){
    vector<int> subsection;

    for(int i = start; i < end; i++){
        if(i < 0)
            subsection.push_back(scan[i + scan.size()]);
        else if(i >= scan.size())
            subsection.push_back(scan[i - scan.size()]);
        else
            subsection.push_back(scan[i]);
    }

    return subsection;
}

bool DoorFinder::isPeakHallway(vector<int> subsection){
    const int THRESHOLD = 750;
    int minimum = subsection[0];
    int maximum = 0;
    for(int i = 0; i < subsection.size(); i++){
        if(subsection[i] < minimum)
            minimum = subsection[i];
        if(subsection[i] > maximum)
            maximum = subsection[i];
    }

    if(maximum - minimum > THRESHOLD)
        return true;
    else
        return false;
}

bool DoorFinder::isSamePoint(DoublePoint p1, DoublePoint p2, float dist){
    return (abs(p1.x - p2.x) <= dist && abs(p1.y - p2.y) <= dist);
}

bool DoorFinder::isSameAngle(float a1, float a2, float tolerance){
    return ((fabs(a1 - a2) <= tolerance) || (fabs(a1 - a2) >= (360 - tolerance)));
}

std::vector<DoublePoint> DoorFinder::averageEstimations(std::vector<DH> dh){
    std::vector<cluster> clusters;

    for(int i = 0; i < dh.size(); i++){

    }
}

std::vector<DoublePoint> DoorFinder::averagePoints(std::vector<DoublePoint> points){
    float DIST_THRESHOLD = 90;
    std::vector<DoublePoint> ret;

    //average the points in the vector first
    for(int i = 0; i < points.size(); i++){
        bool entered = false;
        for(int j = 0; j < ret.size(); j++){
            if(isSamePoint(points[i], ret[j], DIST_THRESHOLD)){
                //cout << "1(" << ret[j].x << ", " << ret[j].y << "), ";
                //cout << "(" << points[i].x << ", " << points[i].y << "), ";
                ret[j].x = (ret[j].x + points[i].x) / 2;
                ret[j].y = (ret[j].y + points[i].y) / 2;
                //cout << "(" << ret[j].x << ", " << ret[j].y << "), " << endl;
                entered = true;
                //break;
            } else {
                //cout << "2(" << ret[j].x << ", " << ret[j].y << "), ";
                //cout << "(" << points[i].x << ", " << points[i].y << "), " << endl;
            }
        }
        if(!entered){
            ret.push_back(points[i]);
            //for(int j = 0; j < ret.size(); j++){
            //    cout << "(" << ret[j].x << ", " << ret[j].y << "), ";
            //}
            //cout << endl;
        }
    }

    cout << ret.size() << endl;

    for(int i = 0; i < ret.size(); i++){
        cout << "(" << ret[i].x << ", " << ret[i].y << "), ";
    }
    cout << endl;

    return ret;
    //look in the occupancy grid for doors we have already seen by looking in a circle around updated vector
}


