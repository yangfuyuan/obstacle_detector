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

#pragma once

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
  void setCenter(double x, double y) { center_.x = x; center_.y = y; }

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
