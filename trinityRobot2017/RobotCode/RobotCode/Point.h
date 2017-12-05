#pragma once

struct Point {
	Point() {}
	Point(int _x, int _y) { x = _x; y = _y; }
	Point(DoublePoint p) { x = static_cast<int>(p.x); y = static_cast<int>(p.y); }
	int x;
	int y;
};