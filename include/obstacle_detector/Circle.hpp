#ifndef CIRCLE_HPP
#define CIRCLE_HPP

#include <cmath>
#include "Point.hpp"
#include "Segment.hpp"

namespace obstacle_detector
{

struct Circle
{
  Point centre;
  double radius;

  Circle() {}
  Circle(const Point& p, const double& r) : centre(p), radius(r) {}
  Circle(const Point& p1, const Point& p2, const Point& p3)
  {
    double a = (pow(norm(p2 - p3), 2) * (p1 - p2) * (p1 - p3)) / (2.0 * pow(normOfCross((p1 - p2), (p2 - p3)), 2));
    double b = (pow(norm(p1 - p3), 2) * (p2 - p1) * (p2 - p3)) / (2.0 * pow(normOfCross((p1 - p2), (p2 - p3)), 2));
    double c = (pow(norm(p1 - p2), 2) * (p3 - p1) * (p3 - p2)) / (2.0 * pow(normOfCross((p1 - p2), (p2 - p3)), 2));

    centre = a * p1 + b * p2 + c * p3;
    radius = distanceBetween(centre, p1);
  }
  Circle(const Segment& s)
  {
    // From equilateral triangle
    radius = 0.577 * distanceBetween(s.first_point, s.last_point);  // sqrt(3)/3 * length
    centre = (s.first_point + s.last_point - radius * s.normal) / 2.0;
  }

  Point projectPointOnto(const Point& p)
  {
    Point point;

    double phi = atan2(p.y - centre.y, p.x - centre.x);
    point.fromPoolarCoords(radius, phi);
    point.x += centre.x;
    point.y += centre.y;

    return point;
  }

  friend std::ostream& operator<<(std::ostream &out, const Circle &c);
};


double distanceBetween(const Circle& c, const Point& p)
{ return norm(p - c.centre) - c.radius; }

Circle merge(const Circle& c1, const Circle& c2)
{
  Segment s(c1.centre, c2.centre);
  Circle c(s);
  c.radius += (c1.radius > c2.radius ? c1.radius : c2.radius);
  return c;
}

std::ostream& operator<<(std::ostream &out, const Circle &c)
{
  out << c.centre.x << "\t" << c.centre.y << "\t" << c.radius;
  return out;
}

}

#endif
