#pragma once

#include <cmath>
#include <ostream>

using std::ostream;
using std::endl;

//this is very comprehensive
//you can do pretty much anything with this point

template<typename ValType>
struct Point2 {
    ValType x, y;

    Point2(ValType x, ValType y) : x(x), y(y) {}
    Point2() : x(0), y(0) {}

    Point2(const Point2<ValType> &p){
        x = p.x;
        y = p.y;
    }

    template <typename T>
        Point2(const Point2<T> &p) {
            x = p.x;
            y = p.y;
        }

    template <typename T>
        Point2& operator=(const Point2<T> &p) {
            x = p.x;
            y = p.y;
            return *this;
        }

    //if index is 0, return x, else y
    template<typename T>
        const T& operator[](const int index) const {
            return index == 0 ? x : y;
        }

    template<typename T>
        T& operator[](const int index) {
            return index == 0 ? x : y;
        }

    bool operator==(const Point2 &p) const {
        return x == p.x && y == p.y;
    }

    bool operator!=(const Point2 &p) const {
        return x != p.x || y != p.y;
    }

    template <typename T>
        Point2& operator+=(const Point2<T> &p) {
            x += p.x;
            y += p.y;
            return *this;
        }

    template <typename U>
        Point2& operator+=(const U &p) {
            x += p;
            y += p;
            return *this;
        }

    template <typename T>
        Point2& operator-=(const Point2<T> &p) {
            x -= p.x;
            y -= p.y;
            return *this;
        }

    template <typename U>
        Point2& operator-=(const U &p) {
            x -= p;
            y -= p;
            return *this;
        }

    template <typename T>
        Point2& operator*=(const Point2<T> &p) {
            x *= p.x;
            y *= p.y;
            return *this;
        }

    template <typename U>
        Point2& operator*=(const U &p) {
            x *= p;
            y *= p;
            return *this;
        }

    template <typename T>
        Point2& operator/=(const Point2<T> &p) {
            x /= p.x;
            y /= p.y;
            return *this;
        }

    template <typename U>
        Point2& operator/=(const U &p) {
            x /= p;
            y /= p;
            return *this;
        }

    template <typename T>
        Point2 operator+(const Point2<T> &p) const {
            Point2 temp(x + p.x, y + p.y);
            return temp;
        }

    template <typename U>
        Point2 operator+(const U &p) const {
            Point2 temp(x + p, y + p);
            return temp;
        }

    template <typename T>
        Point2 operator-(const Point2<T> &p) const {
            Point2 temp(x - p.x, y - p.y);
            return temp;
        }

    template <typename U>
        Point2 operator-(const U &p) const {
            Point2 temp(x - p, y - p);
            return temp;
        }

    template <typename T>
        Point2 operator*(const Point2<T> &p) const {
            Point2 temp(x * p.x, y * p.y);
            return temp;
        }

    template <typename U>
        Point2 operator*(const U &p) const {
            Point2 temp(x * p, y * p);
            return temp;
        }

    template <typename T>
        Point2 operator/(const Point2<T> &p) const {
            Point2 temp(x / p.x, y / p.y);
            return temp;
        }

    template <typename U>
        Point2 operator/(const U &p) const {
            Point2 temp(x / p, y / p);
            return temp;
        }

    friend ostream& operator<<(ostream &os, const Point2<ValType> &p){
        os << p.x << " " << p.y << endl;
        return os;
    }

    ValType magnitude() const { 
        return sqrt(x*x + y*y);
    }
};

typedef Point2<int> Point;
typedef Point2<double> DoublePoint;

//returns with type of first argument
template<typename T, typename U>
static T computeAngle(const Point2<T> &a, const Point2<U> &b) {
    // atan2 returns a value from 0 to PI and -PI to 0 (so 181 degrees would be aproximately -PI)
    return atan2(b.y - a.y, b.x - a.x);
}
