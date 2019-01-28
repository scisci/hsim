
#ifndef HSIM_GEOMETRY_HPP
#define HSIM_GEOMETRY_HPP

#include <Eigen/Geometry>

#include "hsim/Math.hpp"

namespace hsim {

enum GeometryType {
  kBox,
  kSphere,
  kCone
};

class Geometry {
public:
  virtual ~Geometry() {}
  virtual GeometryType Type() const = 0;
  virtual Eigen::AlignedBox<Real, 3> BoundingBox() const = 0;
};


class Box : public Geometry {
public:
  Box(Real width, Real height, Real depth)
  {
    sizes_[0] = width;
    sizes_[1] = height;
    sizes_[2] = depth;
  }
  
  virtual GeometryType Type() const { return GeometryType::kBox; }
  virtual Eigen::AlignedBox<Real, 3> BoundingBox() const
  {
    return Eigen::AlignedBox<Real, 3>(
      Vector3(-sizes_[0] / 2, -sizes_[1] / 2, -sizes_[2] / 2),
      Vector3(sizes_[0] / 2, sizes_[1] / 2, sizes_[2] / 2));
  }
  
  Real Width() const { return sizes_[0]; }
  Real Height() const { return sizes_[1]; }
  Real Depth() const { return sizes_[2]; }
  

private:
  Real sizes_[3];
};


class Sphere : public Geometry {
public:
  Sphere(Real radius)
  : radius_(radius)
  {}
  
  virtual GeometryType Type() const { return GeometryType::kSphere; }
  virtual Eigen::AlignedBox<Real, 3> BoundingBox() const
  {
    return Eigen::AlignedBox<Real, 3>(
      Vector3(-radius_, -radius_, -radius_),
      Vector3(radius_, radius_, radius_));
  }
  
  Real Radius() const { return radius_; }

private:
  Real radius_;
};

class Cone : public Geometry {
public:
  Cone(Real radius, Real height)
  : radius_(radius),
    height_(height)
  {}
  
  virtual GeometryType Type() const { return GeometryType::kCone; }
  virtual Eigen::AlignedBox<Real, 3> BoundingBox() const
  {
    return Eigen::AlignedBox<Real, 3>(
      Vector3(-radius_, 0, -radius_),
      Vector3(radius_, height_, radius_));
  }
  Real Radius() const { return radius_; }
  Real Height() const { return height_; }

private:
  Real radius_;
  Real height_;
};

class MassProperties {
public:
  static Matrix3 ComputeInertia(const Geometry& geometry, Real mass);
  static Matrix3 ComputeInertia(const Box& box, Real mass);
  static Matrix3 ComputeInertia(const Sphere& sphere, Real mass);
  static Matrix3 ComputeInertia(const Cone& cone, Real mass);
  
  static Real ComputeVolume(const Geometry& geometry);
  static Real ComputeVolume(const Box& box);
  static Real ComputeVolume(const Sphere& sphere);
  static Real ComputeVolume(const Cone& cone);
};

} // namespace hsim


#endif // HSIM_GEOMETRY_HPP
