#ifndef POINT_HPP
#define POINT_HPP

#include <cmath>

namespace obstacle_detector
{

struct Point
{
  double x;
  double y;

  Point(double x = 0.0, double y = 0.0) : x(x), y(y) {}

  void fromPoolarCoords(const double& r, const double& phi)
  { x = r * cos(phi);  y = r * sin(phi); }

  friend Point operator+(const Point& p1, const Point& p2)
  { return Point(p1.x + p2.x, p1.y + p2.y); }

  friend Point operator-(const Point& p1, const Point& p2)
  { return Point(p1.x - p2.x, p1.y - p2.y); }

  friend Point operator*(const double& d, const Point& p)
  { return Point(d * p.x, d * p.y); }

  friend Point operator*(const Point& p, const double& d)
  { return Point(d * p.x, d * p.y); }

  friend double operator*(const Point& p1, const Point& p2)
  { return p1.x * p2.x + p1.y * p2.y; }

  friend Point operator/(const Point& p, const double& d)
  { return Point(p.x / d, p.y / d); }
};

double norm(const Point& p)
{ return sqrt(pow(p.x, 2) + pow(p.y, 2)); }

double normOfCross(const Point& p1, const Point& p2)
{ return fabs(p1.x * p2.y - p1.y * p2.x); }

double distanceBetween(const Point& p1, const Point& p2)
{ return norm(p1 - p2); }

double signum(double d)
{ return (d < 0.0) ? -1.0 : 1.0; }

}

#endif
