using namespace std;

class Point {

public:
	Point();
	Point(int _x, int _y);
	int x;
	int y;
	bool operator==(const Point &p)const;
};