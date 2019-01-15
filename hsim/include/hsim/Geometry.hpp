
#ifndef HSIM_GEOMETRY_HPP
#define HSIM_GEOMETRY_HPP

#include <Eigen/Geometry>

#include "hsim/Math.hpp"

namespace hsim {

enum GeometryType {
  kBox,
  kSphere
};

class Geometry {
public:
  virtual ~Geometry() {}
  virtual GeometryType Type() const = 0;
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
  Real Radius() const { return radius_; }

private:
  Real radius_;
};

class MassProperties {
public:
  static Matrix3 ComputeInertia(const Geometry& geometry, Real mass);
  static Matrix3 ComputeInertia(const Box& box, Real mass);
  static Matrix3 ComputeInertia(const Sphere& sphere, Real mass);
  
  static Real ComputeVolume(const Geometry& geometry);
  static Real ComputeVolume(const Box& box);
  static Real ComputeVolume(const Sphere& sphere);
};

} // namespace hsim


#endif // HSIM_GEOMETRY_HPP
