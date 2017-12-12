#include "Point.h"
using namespace std;

Point::Point() {}
Point::Point(int _x, int _y) { x = _x; y = _y; }
Point::Point(DoublePoint p) { x = static_cast<int>(p.x); y = static_cast<int>(p.y); }
Point::operator-(Point other){return Point(x-other.x, y-other.y);}
Point::operator==(Point other){return x == other.x && y == other.y;}
Point::operator!=(Point other){return x != other.x || y != other.y;}
}
