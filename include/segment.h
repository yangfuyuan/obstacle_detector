/*
 * Software License Agreement (BSD License)
 *
 * Copyright (c) 2015, Poznan University of Technology
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 *
 *     * Redistributions of source code must retain the above copyright
 *       notice, this list of conditions and the following disclaimer.
 *     * Redistributions in binary form must reproduce the above copyright
 *       notice, this list of conditions and the following disclaimer in the
 *       documentation and/or other materials provided with the distribution.
 *     * Neither the name of the Willow Garage, Inc. nor the names of its
 *       contributors may be used to endorse or promote products derived from
 *       this software without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE
 * LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
 * CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
 * SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
 * INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
 * CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
 * ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 */

/*
 * Author: Mateusz Przybyla
 */

#ifndef SEGMENT_H
#define SEGMENT_H

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

  void setFirstPoint(double x, double y) { p1_.x = x; p1_.y = y; }
  void setLastPoint(double x, double y) { p2_.x = x; p2_.y = y; }

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

  bool complete_;   // Complete line segment
  bool isolated_;   // Not part of bigger cluster
};

} // namespace obstacle_detector

#endif // SEGMENT_H
