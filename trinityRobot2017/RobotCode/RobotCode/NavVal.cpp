#include "NavVal.h"
#include "constants.h"
float cellType;
int timesScanned;



//constructor
NavVal::NavVal() : cellType(-1), timesScanned(0) {

}

// usage: nv.updateValue(NavVal::WALL);
int NavVal::updateValue(float newCellType) {

	if (this->cellType <= 1) {
		// free to update value
		if (newCellType <= 1) {
			// A routine clear/wall update
			this->cellType = ((this->cellType * timesScanned) + newCellType) / (timesScanned + 1); // update probability
			timesScanned++;
		}
		else { // override current value for new discrete value.
			// maybe some value checks can be put here in the future
			this->cellType = newCellType;
		}
	}
	else if (newCellType >= 2){
		// This value must be a special case already, only change this value if the new cellType is
		// not a unknown, clear or wall
		this->cellType = newCellType;
	}
	else {
		// in this scenario, a cell that is a special value ( >=2 ), is now scanned as a wall or clear.
		// So I guess we ignore this input.
	}
	return 1; // returning 1 usually means everything went well.
	//We can change this to void if nothing bad can ever happen in this function
}

int NavVal::getCellType(void) {
	if (this->cellType >= 0 && this->cellType <= 1) {
		// (basically a clearThreshhold or greater probability will return a WALL and CLEAR otherwise)
		if(this->cellType >= CLEAR_THRESHOLD){
			return 1;
		}else{
			return 0;
		}
	}
	else {
		// must be a special value or unknown
		return this->cellType;
	}
}
