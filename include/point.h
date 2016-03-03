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
