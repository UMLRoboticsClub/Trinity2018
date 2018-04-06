#include "doorfinder.h"
#include "globals.h"
#include "constants.h"
#include "point.h"

#include <vector>
#include <unordered_map>
#include <cmath>
#include <algorithm>
#include <deque>
#include <iostream>

/*********************************
DOOR FINDING CODE

Public functions--
Find doors and hallways
Average - cleans up a laser scan

Todo--
Add looking in the OccGrid for exisitng doors to
compare the ones we're adding to
Figure out how to properly pass OccGrid to 
DoorFinder from MazeMapper

I think I completely commented this but if i didn't
just yell at me

*********************************/

using std::cout;
using std::endl;
using std::deque;
using std::vector;
using std::map;

DoublePoint DoorFinder::prevPos(getRobotPos());

DoorFinder::DoorFinder(OccupancyGrid o){
    //set angle size and other stuff
    numIndex = 0;
    angleSize = 360 / NUM_SECTIONS[numIndex];
    occGrid = o;
}

/*
Find Doors and Hallways
@param scan: deque with the laser scan in the format scan[angle] = dist
@param targetPoints: map of ints (type of target) amd points (locations of targets)

This function will edit target points with the locations of doors and hallways.
*/
map<int, vector<Point>> DoorFinder::findDoorsAndHallways(std::deque<int> scan, map<int, vector<Point>> targetPoints){
    std::vector<DH> all;

    numIndex = 0; //reset index of sections

    //rotate scan to catch edge cases
    for(int i = 0; i < 180; i++){
        //average current scan into sections
        std::vector<int> sections = average(scan, NUM_SECTIONS[numIndex]);
        angleSize = 360 / NUM_SECTIONS[numIndex];

        //sort peaks in data into hallways and doors
        std::vector<int> peaks = findPeaks(sections, 1, sections.size() - 1, true);
        std::vector<int> hallwayPeaks = sortPeaks(scan, sections, peaks, true);
        std::vector<int> doorPeaks = sortPeaks(scan, sections, peaks, false);

        if(peaks.size() == 0 && numIndex < NUM_SECTIONS.size() - 1){ //repeat if no peaks were found
            numIndex++;
            i--;
        } else {
            for(int j = 0; j < hallwayPeaks.size(); j++){ //add hallways
                DH hallway;
                hallway.angle = getAngle(hallwayPeaks[j], i);
                hallway.dist = findDistanceHallway(sections, hallwayPeaks[j]);
                hallway.point = getPoint(hallway.angle, hallway.dist);
                hallway.isHallway = true;
                all.push_back(hallway);
            }
            for(int j = 0; j < doorPeaks.size(); j++){ //add doors
                DH door;
                door.angle = getAngle(doorPeaks[j], i);
                door.dist = findDistanceDoor(scan, sections, doorPeaks[j]);
                door.point = getPoint(door.angle, door.dist);
                door.isHallway = false;
                all.push_back(door);
            }

            //rotate the scan
            double r = scan.back();
            scan.pop_back();
            scan.push_front(r);
            numIndex = 0;
        }
    }
    vector<cluster> clusters = averageEstimations(all); //average everything found


    //put final doors and hallways in the target values list
    for(int i = 0; i < clusters.size(); i++){
        if(clusters[i].size >= 25){
            double percent = clusters[i].averageAngle / 360;
            clusters[i].averagePoint = clusters[i].averagePoint / 10.0 - (getRobotPos() - prevPos)*percent; 
            if(occGrid.getValue(getRobotPos().x + clusters[i].averagePoint.x, getRobotPos().y + clusters[i].averagePoint.y) == WALL)
                clusters[i] = shiftCluster(clusters[i]); //if we think a door/hallway is in a wall :(
            if(clusters[i].hallwayCount >= clusters[i].size * 0.5){
                if(targetPoints.find(HALLWAY) == targetPoints.end())
                    targetPoints.insert(std::make_pair(HALLWAY, vector<Point>()));
                targetPoints[HALLWAY].push_back(clusters[i].averagePoint);
                //occGrid.update(getRobotPos().x + clusters[i].averagePoint.x / 10, getRobotPos().y + clusters[i].averagePoint.y / 10, HALLWAY);
            } else {
                if(targetPoints.find(DOOR) == targetPoints.end())
                    targetPoints.insert(std::make_pair(DOOR, vector<Point>()));
                targetPoints[DOOR].push_back(clusters[i].averagePoint);
                //occGrid.update(getRobotPos().x + clusters[i].averagePoint.x / 10, getRobotPos().y + clusters[i].averagePoint.y / 10, DOOR);
            }
        }
    }
    prevPos = getRobotPos();
    return targetPoints;
}

DoublePoint DoorFinder::getPoint(int angle, float distance){
    DoublePoint point;

    point.x = distance * cos((double) angle * M_PI / 180);
    point.y = distance * sin((double) angle * M_PI / 180);

    return point;
}

/*
If a door or hallway is in a wall, shift it towards the robot position until it is no longer in the wall
@param c: cluster of door/hallway points to shift
*/
cluster DoorFinder::shiftCluster(cluster c){
    //find length between robot and door/hall and dx and dy
    float len = std::sqrt(std::pow((c.averagePoint.x - getRobotPos().x), 2) + std::pow((c.averagePoint.y - getRobotPos().y), 2));
    float dx = (c.averagePoint.x - getRobotPos().x) / len;
    float dy = (c.averagePoint.y - getRobotPos().y) / len;

    for(int i = 0; i < len; i++){ //shift until no longer in wall
        c.averagePoint.x += dx;
        c.averagePoint.y -= dy;

        if(occGrid.getValue(getRobotPos().x + c.averagePoint.x, getRobotPos().y + c.averagePoint.y) != WALL)
            break;
    }

    for(int i = 0; i < 35; i++){ //shift a little extra
        c.averagePoint.x += dx;
        c.averagePoint.y -= dy;

        if(occGrid.getValue(getRobotPos().x + c.averagePoint.x, getRobotPos().y + c.averagePoint.y) == WALL){
            c.averagePoint.x -= dx;
            c.averagePoint.y += dy;
            break;
        }
    }
    return c;
}

/*
Get the angle of a peak, given rotation
@return: angle in degrees
*/
int DoorFinder::getAngle(int peak, int rotation){
    int angle = (peak + 1) * angleSize - angleSize * (3.0/4.0) - (rotation);
    angle -= (int) (getRobotAngle() * 180 / M_PI);
    if(angle > 360)
        angle = angle % 360;
    if(angle < 0)
        angle = 360 + angle;
    return angle;
}

/*
Average/clean up a scan
@param scan: raw laser scan data
@param numSections: number of sections to average into
*/
vector<int> DoorFinder::average(deque<int> scan, int numSections){
    vector<int> averaged(numSections);
    if(scan.size() != 0){
        int numItems = scan.size() / numSections;
        int start = 0;
        int nd = numItems;
        int real = numItems;

        for(int i = 0; i < numSections; i++){
            for(int j = start; j < nd; j++){
                if(scan[j] > 100) //account for points inside the robot
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
        //I should probably do something with this blah
    }
    if(averaged[0] == -1)
        averaged[0] = averaged[averaged.size() - 1];
    return averaged;
}

/*
Find peaks - takes a clean laser scan and finds peaks in the data
@param averaged: clean scan
@param start: index to start looking
@param nd: index to stop looking
@param checkEndpoints: whether to treat the scan as circular (default TRUE)
*/
vector<int> DoorFinder::findPeaks(vector<int> averaged, int start, int nd, bool checkEndpoints = true){
    float margin = 110; //peak must be this far above its neighbors

    vector<int> indices;
    for(int i = start; i < nd; i++){
        if(averaged[i-1] + margin <= averaged[i] && averaged[i+1] + margin <= averaged[i]){
            indices.push_back(i);
        }
    }

    if(checkEndpoints){
        if(averaged[start] + margin <= averaged[start - 1] && averaged[nd] + margin <= averaged[start - 1])
            indices.push_back(start - 1);
        if(averaged[nd - 1] + margin <= averaged[nd] && averaged[start - 1] + margin <= averaged[nd])
            indices.push_back(nd);
    }

    return indices;
}

/*
Find the distance to a peak by looking at the points around it
@param isHallway: if the peak is a hallway, take less of the surrounding point into account
because it will be a bigger spike in the data
*/
int DoorFinder::findDistanceHallway(vector<int> averaged, int peak){
    vector<int> doorDist;
    float ret;

    if(peak == 0) {
        doorDist.push_back(averaged[averaged.size() - 1]);
        //if(!isHallway)
        //    doorDist.push_back(averaged[averaged.size() - 2]);
    } else if(peak == 1) {
        //if(!isHallway)
        //    doorDist.push_back(averaged[averaged.size() - 1]);
        doorDist.push_back(averaged[0]);
    } else {
        //if(!isHallway)
        //    doorDist.push_back(averaged[peak - 2]);
        doorDist.push_back(averaged[peak - 1]);
    }

    if(peak == averaged.size() - 1){
        doorDist.push_back(averaged[0]);
        //if(!isHallway)
        //    doorDist.push_back(averaged[1]);
    } else if(peak == averaged.size() - 2){
        //if(!isHallway)
        //    doorDist.push_back(averaged[0]);
        doorDist.push_back(averaged[averaged.size() - 1]);
    } else {
        doorDist.push_back(averaged[peak + 1]);
        //if(!isHallway)
        //    doorDist.push_back(averaged[peak + 2]);
    }

    for(int i = 0; i < doorDist.size(); i++){
        ret += doorDist[i];
    }
    ret /= doorDist.size();
    return (int) ret;
}

int DoorFinder::findDistanceDoor(deque<int> scan, vector<int> averaged, int peak){
    int THRESHOLD = 200;
    int dist = 0;
    int numSections = averaged.size();
    float sectionWidth = scan.size() / numSections;
    vector<int> subsection = this->getSubsection(scan, (((peak + 1) * sectionWidth) - sectionWidth / 2 - 60), (((peak + 1) * sectionWidth ) - sectionWidth / 2 + 60));

    int max = 0;
    int index = 0;
    for(int i = 0; i < subsection.size(); i++){
        if(subsection[i] > max){
            max = subsection[i];
            index = i;
        }
    }


    int offset = 1;
    bool nd = true;
    for(int i = index; i > 0 && nd; i--){
        if(subsection[i] > 100){
            while(i - offset > 0){
                if(subsection[i - offset] > 100 && subsection[i] - subsection[i - offset] > THRESHOLD){
                    dist = subsection[i - offset];
                    nd = false;
                    break;
                } else {
                    offset++;
                }
            }
        }
        offset = 1;
    }

    offset = 1;
    if(dist == 0){
        nd = true;
        for(int i = index; i < subsection.size(); i++){
            if(subsection[i] > 100){
                while(i + offset < subsection.size()){
                    if(subsection[i + offset] > 100 && subsection[i] - subsection[i + offset] > THRESHOLD){
                        dist = subsection[i + offset];
                        nd = false;
                        break;
                    } else {
                        offset++;
                    }
                }
            }
            offset = 1;
        }
    }
    return dist;

}

/*
Split a vector of peaks into doors and hallways
*/
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

/*
Get a subsection of the raw laser scan
*/
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

/*
Check if a subsection of the raw laser scan is a hallway
*/
bool DoorFinder::isPeakHallway(vector<int> subsection){
    const int THRESHOLD = 415; //threshold of a hallway in mm
    int minimum = subsection[0];
    int maximum = 0;
    bool reset = false; //helps to discount garbage values

    if(minimum < 100)
        reset = true;

    for(int i = 0; i < subsection.size(); i++){
        if(reset && subsection[i] > 100){ //discount garbage
            minimum = subsection[i];
            reset = false;
        }
        if(subsection[i] < minimum && subsection[i] > 100)
            minimum = subsection[i];
        if(subsection[i] > maximum)
            maximum = subsection[i];
    }

    if(maximum - minimum > THRESHOLD)
        return true;
    else
        return false;
}

/*
Whether two angles can be considered the same
*/
bool DoorFinder::isSameAngle(float a1, float a2, float tolerance){
    return ((fabs(a1 - a2) <= tolerance) || (fabs(a1 - a2) >= (360 - tolerance)));
}

/*
Average door/hallway estimations into clusters of what the code thinks are the
same door/hallway. Counts the number of times it was scanned as a door/hallway
to find out what the most likely candidate is
*/
std::vector<cluster> DoorFinder::averageEstimations(std::vector<DH> dh){
    std::vector<cluster> clusters;

    for(int i = 0; i < dh.size(); i++){
        bool found = false;
        for(int j = 0; j < clusters.size(); j++){
            if(isSameAngle(clusters[j].averageAngle, dh[i].angle, angleSize)){
                clusters[j].averageAngle = (clusters[j].averageAngle * clusters[j].size + dh[i].angle) / (clusters[j].size + 1);
                clusters[j].averagePoint.x = (clusters[j].averagePoint.x * clusters[j].size + dh[i].point.x) / (clusters[j].size + 1);
                clusters[j].averagePoint.y = (clusters[j].averagePoint.y * clusters[j].size + dh[i].point.y) / (clusters[j].size + 1);
                clusters[j].size++;
                if(dh[i].isHallway)
                    clusters[j].hallwayCount++;
                found = true;
                break;
            }
        }
        if(!found){
            clusters.push_back(cluster());
            clusters[clusters.size() - 1].averageAngle = dh[i].angle;
            clusters[clusters.size() - 1].averagePoint.x = dh[i].point.x;
            clusters[clusters.size() - 1].averagePoint.y = dh[i].point.y;
            if(dh[i].isHallway)
                clusters[clusters.size() - 1].hallwayCount = 1;
            else
                clusters[clusters.size() - 1].hallwayCount = 0;
            clusters[clusters.size() - 1].size = 1;
        }
    }

    cout << "num clusters: " << clusters.size() << endl;
    for(int i = 0; i < clusters.size(); i++){
        cout << "avg angle: " << clusters[i].averageAngle << endl;
        cout << "avg point: (" << clusters[i].averagePoint.x << ", " << clusters[i].averagePoint.y << ")\n";
        cout << "hallway count: " << clusters[i].hallwayCount << endl;
        cout << "size: " << clusters[i].size << endl << endl;
    }
    return clusters;
}
