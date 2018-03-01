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

using namespace std;

DoorFinder::DoorFinder(){
    //set angle size and other stuff
}

void DoorFinder::findDoorsAndHallways(LaserScanner scan, map<int, vector<Point>>& targetPoints);
    //constants: NUM_SECTIONS,
	//convert the array part of the scan to a deque
	std::deque<double> scanDeque;
	std::vector<DoublePoint> allDoors;
	std::vector<DoublePoint> allHall;
	scanDeque.assign(scan.getData(), scan.getData() + scan.getSize()); //idk if this is right

	//loop through x times
	for(int i = 0; i < scan.getSize(); i++){
		//average current scan into sections
		std::vector<double> sections = average(scanDeque, NUM_SECTIONS);
		//find the peaks of the sections
		std::vector<int> peaks = findPeaks(sections, 1, sections.size() - 1, true);
		if(peaks.size() == 0){ //no peaks were found, let's try again
            // do shit, idk what until we see a real laser scan
		} else {
            //sort peaks into doors vs hallways
            allDoors += pointsFromPeaks(sections, sortPeaks(scanDeque, sections, peaks, false), i);
            allHall += pointsFromPeaks(sections, sortPeaks(scanDeque, sections, peaks, true), i);
		}
		//reset scan variables?
		//rotate deque
		double r = scanDeque.back();
		scanDeque.pop_back();
		scanDeque.push_front(r);
	}
	//average all estimations to find out which are of the same door/hallway
	allDoors = averagePoints(allDoors);
	allHall = averagePoints(allHall);

	//put final results into the important values list
}

std::vector<DoublePoint> averagePoints(std::vector<DoublePoint> points){
    //average the points in the vector first

    //look in the occupancy grid for doors we have already seen by looking in a circle around updated vector
}


vector<double> DoorFinder::average(deque<double> scan, int numSections){
    vector<double> averaged;
    if(scan.size() != 0){
        int numItems = scan.size() / numSections;
        int start = 0;
        int nd = numItems;

        for(int i = 0; i < numParts, i++){
            for(int j = start; j < nd; j++){
                averaged.at(i) += scan[j];
            }
            averaged.at(i) /= numItems;
            start = nd;
            nd += numItems;
        }
    } else {
        //ooh an error
    }
    return averaged;
}
vector<int> DoorFinder::findPeaks(vector<double> averaged, int start, int nd, bool checkEndpoints){
    float margin = 0.5; //peak must be this far above its neighbors
    vector<int> indices;
    for(int i = start, i < end; i++){
        if(averaged[i-1] + margin <= averaged[i] && averaged[i+1] + margin <= array[i]){
            indices.push_back(i);
        }
    }

    if(checkEndpoints){
        if(averaged[start] + margin <= averaged[start - 1] && averaged[end] + margin <= averaged[start - 1])
            indices.push_back(start - 1);
        if(averaged[end - 1] + margin <= averaged[end] && averaged[start - 1] + margin <= averaged[end])
            indices.push_back(end);
    }

    return indices;
}
vector<int> DoorFinder::sortPeaks(deque<double> scan, vector<double> averaged, vector<int> peaks, bool isHallway){
    vector<int> sortedPeaks;
    int numSections = averaged.size();
    float sectionWidth = scan.size() / numSections;

    for(i = 0; i < peaks.size(); i++){
        vector<double> subsection = this->getSubsection(scan, (peaks[i] * sectionWidth - 20), (peaks[i] * sectionWidth + 20));
        if(this->isPeakHallway(subsection) && isHallway)
            sortedPeaks.push_back(peaks[i]);
        else if(!this->isPeakHallway(subsection) && !isHallway)
            sortedPeaks.push_back(peaks[i]);
    }
    return sortedPeaks;
}
vector<DoublePoint> DoorFinder::pointsFromPeaks(vector<double> averaged, vector<int> peaks, int rotation){
    vector<DoublePoint> points;
    vector<double> distances;

    for(i = 0; i < peaks.size(); i++){
        distances.push_back(this->findDistance(averaged, peaks[i]));
        DoublePoint point;

        angle = (peaks[i] + 1) * angleSize - angleSize * (3.0/4.0) - (0.5 * rotation);
        if(angle > 360)
            angle = angle % 360;
        if(angle < 0)
            angle = 360 + angle;

        //finish after seeing a real scan...
    }

    return points;
}

double DoorFinder::findDistance(vector<double> averaged, int peak){
    //TODO write when we get a real scan :(
    return 0;
}

void DoorFinder::updateAngleSize(){
    angleSize = 0; //TODO UPDATE
}

vector<double> DoorFinder::getSubsection(deque<double> scan, int start, int end){
    vector<double> subsection;

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

bool DoorFinder::isPeakHallway(vector<double> subsection){
    const int THRESHOLD = 10;
    double minimum = subsection[0];
    double maximum = 0;
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

