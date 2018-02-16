#pragma once
#include <cmath>

struct DoublePoint {
	DoublePoint() : x(0), y(0) {};
	DoublePoint(double x, double y) : x(x), y(y) {}
	double x, y;

	static float computeAngle(DoublePoint firstPoint, DoublePoint secondPoint) {
		// atan2 returns a value from 0 to PI and -PI to 0 (so 181 degrees would be aproximately -PI)
		return atan2(firstPoint.y - secondPoint.y, firstPoint.x - secondPoint.x);
	}
};


template<typename T = double>
struct Point2 {
		T x, y;

	Point2<T>(T _x, T _y) : x(_x), y(_y) {}
	Point2() : x(0), y(0) {}

	Point2(const Point2 &p) {
		x = p.x;
		y = p.y;
	}

	template <typename U>
	Point2& operator=(const Point2<U> &p) {
		x = p.x;
		y = p.y;
		return *this;
	}

	template <typename U>
	bool operator==(const Point2<U> &p) {
		return x == p.x && y == p.y;
	}

	template <typename U>
	bool operator!=(const Point2<U> &p) {
		return x != p.x || y != p.y;
	}

	template <typename U>
	Point2& operator+=(const Point2<U> &p) {
		x += p.x;
		y += p.y;
		return *this;
	}

	template <typename U>
	Point2& operator-=(const Point2<U> &p) {
		x -= p.x;
		y -= p.y;
		return *this;
	}

	template <typename U>
	Point2& operator*=(const Point2<U> &p) {
		x *= p.x;
		y *= p.y;
		return *this;
	}

	template <typename U>
	Point2& operator/=(const Point2<U> &p) {
		x /= p.x;
		y /= p.y;
		return *this;
	}

	template <typename U>
	Point2 operator+(const Point2<U> &p) {
		Point2 temp(x + p.x, y + p.y);
		return temp;
	}

	template <typename U>
	Point2 operator-(const Point2<U> &p) {
		Point2 temp(x - p.x, y - p.y);
		return temp;
	}

	template <typename U>
	Point2 operator*(const Point2<U> &p) {
		Point2 temp(x * p.x, y * p.y);
		return temp;
	}

	template <typename U>
	Point2 operator/(const Point2<U> &p) {
		Point2 temp(x / p.x, y / p.y);
		return temp;
	}
};