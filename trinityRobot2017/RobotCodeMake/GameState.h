#pragma once

#include <vector>
#include "Constants.h"

using namespace std;
//right now we don't have baby as a target type wut
//we need baby to be atarget type
struct GameState {

	GameState(int _levelOfCompetition = 1, int _numCandlesExtinguished = 0, bool _babySaved = false, bool _babyObtained = false, bool _safeZoneFound = false,  bool _inRoom = true, bool _secondArena = false){
		levelOfCompetition = _levelOfCompetition;
		numCandlesExtinguished = _numCandlesExtinguished;
		babySaved = _babySaved;
		babyObtained = _babyObtained;
		safeZoneFound = _safeZoneFound;
		inRoom = _inRoom;
		secondArena = _secondArena;
	}

	//in level 3, there are a total of 3 candles.  1 in B, lit at the start,
	//and 2 in A - one lit after 90 seconds, one lit after 120 seconds
	//as far as I can tell there's only one candle in levels 1 and 2

	//when completely done with arena B might be worth it to "wall it off" so we don't accidentally go back there cuz we're stupid.
	//though that shouldn't be an issue.  I hope.  Maybe it would be.

	//avoid going into rooms when on level 3 and in arena 1.  Do we need a thing
	//to specify NOT to go into room?  How to do, how to do...

	//we need to know more info....  this is a pain in the ass

	int levelOfCompetition;
	int numCandlesExtinguished;
	bool babySaved;
	bool babyObtained;
	bool safeZoneFound;
	bool inRoom;
	bool secondArena;

	vector<int> getTargetType() {
		//returns what kind of objects (in order or priority) to look for in the important object map before resorting to exploring
		vector<int> targets;
		//if we are level 1 or 2 and have extinguished the candle
		if(levelOfCompetition != 3 && numCandlesExtinguished > 0){
			targets.push_back(START_ZONE);
			return targets;
		}

		//level 3 and all candles extinguished
		if(levelOfCompetition == 3 && numCandlesExtinguished == 3){
			targets.push_back(START_ZONE);
			return targets;
		}

		//we have extinguished the one candle in arena B, return to A
		if(levelOfCompetition == 3 && numCandlesExtinguished == 1 && secondArena){
			targets.push_back(HALLWAY);
			return targets;
		}

		if (levelOfCompetition != 3 || babySaved) {
			targets.push_back(FLAME);
			targets.push_back(CANDLE);
			if(!inRoom)
				targets.push_back(DOOR);
			else
				targets.push_back(EXPLORED_DOOR);
			return targets;
		}

		//level 3, baby not saved
		if (babyObtained) {
			//we have the baby we just need to shove it out the window
			targets.push_back(SAFE_ZONE);
			return targets;
		}
		//we have not found the baby
		else {
			if (!secondArena)
				targets.push_back(HALLWAY);
			else {
				targets.push_back(RED_SIDE_CRADLE);
				targets.push_back(BLUE_SIDE_CRADLE);
				targets.push_back(GREEN_SIDE_CRADLE);
				if(!inRoom)
					targets.push_back(DOOR);
				else
					targets.push_back(EXPLORED_DOOR);
			}
		}
		return targets;
	}
};
