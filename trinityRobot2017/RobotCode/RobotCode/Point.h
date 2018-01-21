#pragma once
using namespace std;
#include "DoublePoint.h"

struct Point {
	Point();
	Point(int _x, int _y);
	Point(DoublePoint p);
	Point operator-(Point other);
	Point operator+(Point other);
	bool operator==(const Point other) const;
	bool operator!=(Point other);
	int x;
	int y;
};
