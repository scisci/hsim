
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
  virtual Real Volume() const = 0;
  virtual Matrix3 ComputeInertia(Real mass) const = 0;
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
  virtual Real Volume() const { return sizes_[0] * sizes_[1] * sizes_[2]; }
  virtual Matrix3 ComputeInertia(Real mass) const
  {
    Matrix3 inertia = Matrix3::Identity();
    inertia(0, 0) = mass / 12.0 * (std::pow(sizes_[1], 2) + std::pow(sizes_[2], 2));
    inertia(1, 1) = mass / 12.0 * (std::pow(sizes_[0], 2) + std::pow(sizes_[2], 2));
    inertia(2, 2) = mass / 12.0 * (std::pow(sizes_[0], 2) + std::pow(sizes_[1], 2));
    return inertia;
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
  virtual Real Volume() const { return M_PI * 4.0 / 3.0 * std::pow(radius_, 3); }
  virtual Matrix3 ComputeInertia(Real mass) const
  {
    Matrix3 inertia = Matrix3::Identity();
    inertia(0, 0) = 2.0 / 5.0 * mass * std::pow(radius_, 2);
    inertia(1, 1) = inertia(0, 0);
    inertia(2, 2) = inertia(0, 0);
    return inertia;
  }
  
  Real Radius() const { return radius_; }

private:
  Real radius_;
};

} // namespace hsim


#endif // HSIM_GEOMETRY_HPP
