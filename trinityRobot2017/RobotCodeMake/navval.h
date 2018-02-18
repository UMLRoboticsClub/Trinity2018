#ifndef NAV_VAL_H
#define NAV_VAL_H

class NavVal {
    public:
        NavVal();

        int updateValue(float newCellType);
        int getCellType() const;

    private:
        float cellType;
        int timesScanned;
};

#endif
