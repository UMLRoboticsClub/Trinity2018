#include "Point.h"
using namespace std;

Point::Point() {}
Point::Point(int _x, int _y) { this->x = _x; this->y = _y; }

bool Point::operator==(const Point &p)const {
	if (p.x == this->x && p.y == this->y) {
		return true;
	}
	else {
		return false;
	}
}
