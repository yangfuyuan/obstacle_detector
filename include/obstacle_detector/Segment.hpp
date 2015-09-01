#ifndef SEGMENT_HPP
#define SEGMENT_HPP

#include <cmath>
#include <ostream>
#include "Point.hpp"

namespace obstacle_detector
{

struct Segment
{
  Point first_point, last_point, normal;  // normal is a perpendicular versor pointing to open space
  double A, B, C;

  Segment() {}
  Segment(const Point& p1, const Point& p2) : first_point(p1), last_point(p2)
  {
    A = p2.y - p1.y;
    B = p1.x - p2.x;
    C = p2.x * p1.y - p2.y * p1.x;

    normal = Point(-A, -B) / norm(Point(A, B));
    normalize();
  }

  void normalize()
  {
    double mu = -signum(C) / sqrt(A * A + B * B);
    A *= mu;    B *= mu;    C *= mu;
  }

  Segment getPerpendicular(const Point& p)
  {
    Segment s;
    s.A = -B;
    s.B =  A;
    s.C = -s.A * p.x - s.B * p.y;

    s.normalize();

    return s;
  }

  Segment getBisector()
  {
    Point centre = (first_point + last_point) / 2.0;
    return getPerpendicular(centre);
  }

  Point getProjectedPoint(const Point& p)
  {
    Segment s = getPerpendicular(p);
    return getIntersectionPoint(s);
  }

  Point getIntersectionPoint(const Segment& s)
  {
    Point intersection;
    double D;

    if (D = (A * s.B - s.A * B))
    {
      intersection.x = (s.C * B - C * s.B) / D;
      intersection.y = (s.A * C - A * s.C) / D;
    }

    return intersection;
  }

  friend bool operator==(const Segment& s1, const Segment& s2);
  friend std::ostream& operator<<(std::ostream &out, const Segment &s);
};


double distanceBetween(const Segment& s, const Point& p)
{
  if (s.A * s.A + s.B * s.B == 1.0)
    return fabs(s.A * p.x + s.B * p.y + s.C);
  else
    return fabs(s.A * p.x + s.B * p.y + s.C) / sqrt(s.A * s.A + s.B * s.B);
}

Segment merge(const Segment& segm1, const Segment& segm2)
{
  Segment s1, s2;

  // Segments must be provided in counter-clokwise order
  if (distanceBetween(segm1.last_point, segm2.first_point) < distanceBetween(segm1.first_point, segm2.last_point))
    s1 = segm1, s2 = segm2;
  else
    s1 = segm2, s2 = segm1;

  Segment s((s1.first_point + s1.last_point) / 2.0, (s2.first_point + s2.last_point) / 2.0);
  s.normalize();

  s.first_point = s.getProjectedPoint(s1.first_point);
  s.last_point  = s.getProjectedPoint(s2.last_point);

  return s;
}

bool operator==(const Segment& s1, const Segment& s2)
{
  extern double p_max_merge_distance;

  if (distanceBetween(s1.last_point, s2.first_point) < p_max_merge_distance)
  {
    Segment s = merge(s1, s2);

    if (distanceBetween(s, s1.first_point) < p_max_merge_distance &&
        distanceBetween(s, s1.last_point)  < p_max_merge_distance &&
        distanceBetween(s, s2.first_point) < p_max_merge_distance &&
        distanceBetween(s, s2.last_point)  < p_max_merge_distance)
      return true;
  }

  return false;
}

std::ostream& operator<<(std::ostream &out, const Segment &s)
{
  out << s.first_point.x << "\t" << s.first_point.y << "\t" << s.last_point.x << "\t" << s.last_point.y;
  return out;
}

}

#endif
