
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
  virtual AlignedBox BoundingBox() const = 0;
  virtual std::unique_ptr<Geometry> Expand(Real offset) const = 0;
};


class Box : public Geometry {
public:
  Box(Real width, Real height, Real depth)
  {
    half_sizes_[0] = width / 2;
    half_sizes_[1] = height / 2;
    half_sizes_[2] = depth / 2;
  }
  
  virtual GeometryType Type() const { return GeometryType::kBox; }
  virtual AlignedBox BoundingBox() const
  {
    return AlignedBox(
      Vector3(-half_sizes_[0], -half_sizes_[1], -half_sizes_[2]),
      Vector3(half_sizes_[0], half_sizes_[1], half_sizes_[2]));
  }
  
  virtual std::unique_ptr<Geometry> Expand(Real offset) const
  {
    return std::unique_ptr<Geometry>(new Box(
      Width() + offset * 2, Height() + offset * 2, Depth() + offset * 2));
  }
  
  Real Width() const { return half_sizes_[0] * 2; }
  Real Height() const { return half_sizes_[1] * 2; }
  Real Depth() const { return half_sizes_[2] * 2; }
  

private:
  Real half_sizes_[3];
};


class Sphere : public Geometry {
public:
  Sphere(Real radius)
  : radius_(radius)
  {}
  
  virtual GeometryType Type() const { return GeometryType::kSphere; }
  virtual AlignedBox BoundingBox() const
  {
    return AlignedBox(
      Vector3(-radius_, -radius_, -radius_),
      Vector3(radius_, radius_, radius_));
  }
  virtual std::unique_ptr<Geometry> Expand(Real offset) const
  {
    return std::unique_ptr<Geometry>(new Sphere(radius_ + offset));
  }
  
  Real Radius() const { return radius_; }

private:
  Real radius_;
};

class Cone : public Geometry {
public:
  Cone(Real radius, Real height)
  : radius_(radius),
    half_height_(height / 2)
  {}
  
  virtual GeometryType Type() const { return GeometryType::kCone; }
  virtual AlignedBox BoundingBox() const
  {
    return AlignedBox(
      Vector3(-radius_, -half_height_, -radius_),
      Vector3(radius_, half_height_, radius_));
  }
  virtual std::unique_ptr<Geometry> Expand(Real offset) const
  {
    return std::unique_ptr<Geometry>(new Cone(
      radius_ + offset, Height() + offset * 2));
  }
  Real Radius() const { return radius_; }
  Real Height() const { return half_height_ * 2; }

private:
  Real radius_;
  Real half_height_;
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
