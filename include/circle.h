#ifndef CIRCLE_HPP
#define CIRCLE_HPP

#include <list>
#include <cmath>
#include <cassert>

#include "point.h"
#include "segment.h"

namespace obstacle_detector
{

class Circle
{
public:
  Circle(const Point& p = Point(), const double r = 0.0) : center_(p), radius_(r) {
    assert(radius_ >= 0.0);
  }

  /*
   * This creates a circle by taking the segment as a
   * base of equilateral triangle. The circle is circumscribed
   * on this triangle.
   */
  Circle(const Segment& s) {
    radius_ = 0.5773502 * s.length();  // sqrt(3)/3 * length
    center_ = (s.first_point() + s.last_point() - radius_ * s.normal()) / 2.0;
  }

  void setRadius(double r) { radius_ = r; }
  Point center() const { return center_; }
  double radius() const { return radius_; }
  double distanceTo(const Point& p) { return (p - center_).length() - radius_; }
  std::list<Point>& point_set() { return point_set_; }

  friend std::ostream& operator<<(std::ostream& out, const Circle& c)
  { out << c.center_ << " " << c.radius_; return out; }

private:
  Point center_;
  double radius_;
  std::list<Point> point_set_;
};

//Circle(const Point& p1, const Point& p2, const Point& p3) {
//  double a = (p2 - p3).lengthSquared() * (p1 - p2).dot(p1 - p3) / (2.0 * pow((p1 - p2).cross(p2 - p3), 2.0));
//  double b = (p1 - p3).lengthSquared() * (p2 - p1).dot(p2 - p3) / (2.0 * pow((p1 - p2).cross(p2 - p3), 2.0));
//  double c = (p1 - p2).lengthSquared() * (p3 - p1).dot(p3 - p2) / (2.0 * pow((p1 - p2).cross(p2 - p3), 2.0));
//  center_ = a * p1 + b * p2 + c * p3;
//  radius_ = distanceBetween(center_, p1);
//}

} // namespace obstacle_detector

#endif
