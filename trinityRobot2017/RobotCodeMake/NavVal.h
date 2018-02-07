#ifndef NAV_VAL_H
#define NAV_VAL_H

class NavVal {


public:
	//constructor
	NavVal();

	int updateValue(float newCellType);
	int getCellType(void);

private:

	float cellType;
	int timesScanned;

};


#endif
