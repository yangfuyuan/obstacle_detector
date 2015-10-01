#ifndef SEGMENT_HPP
#define SEGMENT_HPP

#include <list>
#include <cmath>
#include <cassert>
#include <iostream>

#include "point.h"

namespace obstacle_detector
{

class Segment
{
public:
  Segment(const Point& p1 = Point(), const Point& p2 = Point()) : p1_(p1), p2_(p2) {
    assert(p1 != p2);

    if (p1.cross(p2) < 0.0) // Swap if not counter-clockwise.
      p1_ = p2, p2_ = p1;
  }

  double length() const { return (p2_ - p1_).length(); }
  Point normal() const { return (p2_ - p1_).perpendicular().normalize(); }
  Point first_point() const { return p1_; }
  Point last_point() const { return p2_; }
  Point projection(const Point& p) const {
    Point a = p2_ - p1_;
    Point b = p - p1_;
    return p1_ + a.dot(b) * a / a.lengthSquared();
  }
  std::list<Point>& point_set() { return point_set_; }
  double distanceTo(const Point& p) const { return (p - projection(p)).length(); }

  friend std::ostream& operator<<(std::ostream& out, const Segment& s)
  { out << s.p1_ << " " << s.p2_; return out; }

private:
  Point p1_;
  Point p2_;
  std::list<Point> point_set_;
};

} // namespace obstacle_detector

#endif
