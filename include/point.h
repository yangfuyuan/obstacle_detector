#ifndef POINT_H
#define POINT_H

#include <cmath>
#include <cassert>
#include <iostream>

namespace obstacle_detector
{

double signum(double x) { return (x < 0.0) ? -1.0 : 1.0; }
double abs(double x) { return (x < 0.0) ? -x : x; }
double max(double x, double y) { return (x > y) ? x : y; }
const double pi = 3.14159265;

class Point
{
public:
  Point(double x = 0.0f, double y = 0.0f) : x(x), y(y) {}
  Point(const Point& p) : x(p.x), y(p.y) {}

  double length()        const { return sqrt(pow(x, 2.0) + pow(y, 2.0)); }
  double lengthSquared() const { return pow(x, 2.0) + pow(y, 2.0); }
  double angle()         const { return atan2(y, x); }
  double angleDeg()      const { return 180.0f * atan2(y, x) / pi; }
  double dot(const Point& p)   const { return x * p.x + y * p.y; }
  double cross(const Point& p) const { return x * p.y - y * p.x; }

  Point& normalize() {
    double L = length();
    if (L > 0.0f)
      x /= L, y /= L;
    return *this;
  }

  /*
   * Returns a vector as if it was reflected from the surface
   * which normal vector is given by parameter. normal is assumed
   * to be normalized.
   */
  Point reflected(const Point& normal) const { return *this - 2.0f * normal * (normal.dot(*this)); }
  Point perpendicular() const { return Point(-y, x); }
  static Point fromPoolarCoords(const double r, const double phi) { return Point(r * cos(phi), r * sin(phi)); }

  friend Point operator+ (const Point& p1, const Point& p2) { return Point(p1.x + p2.x, p1.y + p2.y); }
  friend Point operator- (const Point& p1, const Point& p2) { return Point(p1.x - p2.x, p1.y - p2.y); }
  friend Point operator* (const double f, const Point& p)  { return Point(f * p.x, f * p.y); }
  friend Point operator* (const Point& p, const double f)  { return Point(f * p.x, f * p.y); }
  friend Point operator/ (const Point& p, const double f)  { assert(f != 0.0f); return Point(p.x / f, p.y / f); }

  Point operator- () { return Point(-x, -y); }
  Point operator+ () { return Point( x,  y); }

  Point& operator=  (const Point& p) { if (this != &p) { x = p.x; y = p.y; } return *this; }
  Point& operator+= (const Point& p) { x += p.x; y += p.y; return *this; }
  Point& operator-= (const Point& p) { x -= p.x; y -= p.y; return *this; }

  friend bool operator== (const Point& p1, const Point& p2) { return (p1.x == p2.x && p1.y == p2.y); }
  friend bool operator!= (const Point& p1, const Point& p2) { return !(p1 == p2); }
  friend bool operator<  (const Point& p1, const Point& p2) { return (p1.lengthSquared() < p2.lengthSquared()); }
  friend bool operator<= (const Point& p1, const Point& p2) { return (p1.lengthSquared() <= p2.lengthSquared()); }
  friend bool operator>  (const Point& p1, const Point& p2) { return (p1.lengthSquared() > p2.lengthSquared()); }
  friend bool operator>= (const Point& p1, const Point& p2) { return (p1.lengthSquared() >= p2.lengthSquared()); }
  friend bool operator!  (const Point& v) { return (v.x == 0.0f && v.y == 0.0f); }

  friend std::ostream& operator<<(std::ostream& out, const Point& p)
  { out << "(" << p.x << ", " << p.y << ")"; return out; }

  double x;
  double y;
};

} // namespace obstacle_detector

#endif // POINT_H
