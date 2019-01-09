
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
  : width_(width),
    height_(height),
    depth_(depth)
  {}
  
  virtual GeometryType Type() const { return GeometryType::kBox; }
  Real Width() const { return width_; }
  Real Height() const { return height_; }
  Real Depth() const { return depth_; }

private:
  Real width_;
  Real height_;
  Real depth_;
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

} // namespace hsim


#endif HSIM_GEOMETRY_HPP
