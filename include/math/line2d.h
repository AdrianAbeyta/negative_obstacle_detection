//========================================================================
//  This software is free: you can redistribute it and/or modify
//  it under the terms of the GNU Lesser General Public License Version 3,
//  as published by the Free Software Foundation.
//
//  This software is distributed in the hope that it will be useful,
//  but WITHOUT ANY WARRANTY; without even the implied warranty of
//  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
//  GNU Lesser General Public License for more details.
//
//  You should have received a copy of the GNU Lesser General Public License
//  Version 3 in the file COPYING that came with this distribution.
//  If not, see <http://www.gnu.org/licenses/>.
//========================================================================
/*!
\file    line2d.h
\brief   2D Line Segment representation
\author  Joydeep Biswas, (C) 2018
*/
//========================================================================

#include <algorithm>
#include <vector>

#include "eigen3/Eigen/Dense"
#include "eigen3/Eigen/Geometry"

#include "../math/geometry.h"

#ifndef LINE2D_H
#define LINE2D_H


namespace geometry {

template <typename T>
struct Line {
  typedef Eigen::Matrix<T, 2, 1> Vector2T;

  Vector2T p0;
  Vector2T p1;
  Line() {}
  Line(const Vector2T& p0,
       const Vector2T& p1) : p0(p0), p1(p1) {}
  Line(const T& x0, const T& y0, const T& x1, const T& y1) :
      p0(x0, y0), p1(x1, y1) {}

  void Set(const Vector2T& p0_new, const Vector2T& p1_new) {
    p0 = p0_new;
    p1 = p1_new;
  }

  T Length() const {
    return (p0 - p1).norm();
  }

  T SqLength() const {
    return (p0 - p1).squaredNorm();
  }

  Vector2T Dir() const {
    return (p1 - p0).normalized();
  }

  Vector2T UnitNormal() const {
    const Vector2T d = Dir();
    return Vector2T(-d.y(), d.x());
  }

  T ClosestApproach(const Vector2T& p2, const Vector2T& p3) const {
    const Vector2T d = Dir();
    const T l = Length();
    const Vector2T p20 = p2 - p0;
    const Vector2T p30 = p3 - p0;
    const T p2d = d.dot(p20);
    const T p3d = d.dot(p30);
    if (p2d <= T(0) && p3d <= T(0)) {
      return std::min<T>(p20.norm(), p30.norm());
    } else if (p2d >= l && p3d >= l) {
      return std::min<T>((p2 - p1).norm(), (p3 - p1).norm());
    }
    const Vector2T n = UnitNormal();
    return std::min<T>(std::abs<T>(n.dot(p20)), std::abs<T>(n.dot(p30)));
  }

  T ClosestApproach(const Line<T>& l) const {
    return ClosestApproach(l.p0, l.p1);
  }

  bool CloserThan(const Vector2T& p2,
                  const Vector2T& p3,
                  const T& margin) const {
    // Bounding-box broad phase check.
    if (std::min(p0.x(), p1.x()) > std::max(p2.x(), p3.x()) + margin ||
        std::max(p0.x(), p1.x()) < std::min(p2.x(), p3.x()) - margin ||
        std::min(p0.y(), p1.y()) > std::max(p2.y(), p3.y()) + margin ||
        std::max(p0.y(), p1.y()) < std::min(p2.y(), p3.y()) - margin) {
      return false;
    }
    if (Intersects(p2, p3)) return true;
    return (ClosestApproach(p2, p3) < margin);
  }

  bool Crosses(const Vector2T& p2,
               const Vector2T& p3) const {
    // Bounding-box broad phase check.
    if (std::min(p0.x(), p1.x()) > std::max(p2.x(), p3.x())) return false;
    if (std::max(p0.x(), p1.x()) < std::min(p2.x(), p3.x())) return false;
    if (std::min(p0.y(), p1.y()) > std::max(p2.y(), p3.y())) return false;
    if (std::max(p0.y(), p1.y()) < std::min(p2.y(), p3.y())) return false;
    // Narrow-phase check.
    const Vector2T d1 = p1 - p0;
    const Vector2T d2 = p3 - p2;
    /*
    if (Cross<T>(d1, p3 - p0) * Cross<T>(d1, p2 - p0) > -kEpsilon) return false;
    if (Cross<T>(d2, p1 - p2) * Cross<T>(d2, p0 - p2) > -kEpsilon) return false;
    */
    // Okay, the line segments definitely intersect.
    return (Cross<T>(d1, p3 - p0) * Cross<T>(d1, p2 - p0) < -kEpsilon) &&
        (Cross<T>(d2, p1 - p2) * Cross<T>(d2, p0 - p2) < -kEpsilon);
  }

  bool Intersects(const Vector2T& p2,
                  const Vector2T& p3) const {
    // Bounding-box broad phase check.
    if (std::min(p0.x(), p1.x()) > std::max(p2.x(), p3.x())) return false;
    if (std::max(p0.x(), p1.x()) < std::min(p2.x(), p3.x())) return false;
    if (std::min(p0.y(), p1.y()) > std::max(p2.y(), p3.y())) return false;
    if (std::max(p0.y(), p1.y()) < std::min(p2.y(), p3.y())) return false;
    // Narrow-phase check.
    const Vector2T d1 = p1 - p0;
    const Vector2T d2 = p3 - p2;
    if (Cross<T>(d1, p3 - p0) * Cross<T>(d1, p2 - p0) > 0.0) return false;
    if (Cross<T>(d2, p1 - p2) * Cross<T>(d2, p0 - p2) > 0.0) return false;
    // Okay, the line segments definitely intersect.
    return true;
  }

  bool Intersection(const Vector2T& p2,
                    const Vector2T& p3,
                    Vector2T* intersection) const {
    // Bounding-box broad phase check.
    if (std::min(p0.x(), p1.x()) > std::max(p2.x(), p3.x())) return false;
    if (std::max(p0.x(), p1.x()) < std::min(p2.x(), p3.x())) return false;
    if (std::min(p0.y(), p1.y()) > std::max(p2.y(), p3.y())) return false;
    if (std::max(p0.y(), p1.y()) < std::min(p2.y(), p3.y())) return false;
    // Narrow-phase check.
    const Vector2T d1 = p1 - p0;
    const Vector2T d2 = p3 - p2;
    if (Cross<T>(d1, p3 - p0) * Cross<T>(d1, p2 - p0) > 0.0) return false;
    if (Cross<T>(d2, p1 - p2) * Cross<T>(d2, p0 - p2) > 0.0) return false;
    // Okay, the line segments definitely intersect.
    const T d = Cross<T>(d2, -d1);

    if (d == T(0)) { return false; }

    const T tb = Cross<T>(p0 -p2, p0-p1) / d;
    *intersection = p2 + tb * d2;
    return true;
  }

  bool Intersection(const Line<T>& l2,
                    Vector2T* intersection) const {
    return Intersection(l2.p0, l2.p1, intersection);
  }

  bool Intersects(const Line<T>& l2) const {
    return Intersects(l2.p0, l2.p1);
  }

  bool Crosses(const Line<T>& l2) const {
    return Crosses(l2.p0, l2.p1);
  }

  bool RayIntersects(const Vector2T& p, const Vector2T& dir) const {
    Vector2T v0 = p0 - p;
    Vector2T v1 = p1 - p;
    if (Cross(v0, v1) < 0.0) {
      std::swap(v0, v1);
    }
    return (Cross(v0, dir) >= 0.0  && Cross(v1, dir) <= 0.0);
  }

  bool Touches(const Vector2T& p) {
    const Vector2T v0 = p0 - p;
    const Vector2T v1 = p1 - p;
    return (Cross(v0, v1) == 0.0 && v0.dot(v1) < 0.0);
  }

  Vector2T RayIntersection(const Vector2T& p, const Vector2T& dir) const {
    const Vector2T p0_prime = p0 - p;
    const Vector2T p1_prime = p1 - p;
    const T alpha = Cross<T>(p1_prime, p0_prime) / Cross<T>(dir, p0 - p1);
    return (p + alpha * dir);
  }

  EIGEN_MAKE_ALIGNED_OPERATOR_NEW
};

typedef Line<float> line2f;
typedef Line<double> line2d;
typedef Line<int> line2i;

}  // namespace geometry


#endif   // LINE2D_H
