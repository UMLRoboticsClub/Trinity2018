#ifndef NAV_VAL_H
#define NAV_VAL_H

class NavVal {


public:
	//constructor
	NavVal();

	int updateValue(float newCellType);
	int getCellType(void);

	// Not sure what this does
	float static const clearThreshold;

	// Constant cell types, used as parameters to updateValue().
	// enum can't be floats so...
	float static const UNKNOWN;
	float static const CLEAR;
	float static const WALL;
	float static const FLAME;
	float static const CANDLE;
	float static const EXTINGUISHED;
	float static const SAFE_ZONE;
	float static const RED_SIDE_CANDLE;
	float static const BLUE_SIDE_CANDLE;
	float static const GREEN_SIDE_CANDLE;

private:

	float cellType;
	int timesScanned;

};


#endif
