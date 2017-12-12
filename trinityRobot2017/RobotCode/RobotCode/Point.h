using namespace std;

struct Point {
	Point() {}
	Point(int _x, int _y);
	Point(DoublePoint p);
	operator-(Point other);
	operator==(Point other);
	operator!=(Point other);
	int x;
	int y;
};
