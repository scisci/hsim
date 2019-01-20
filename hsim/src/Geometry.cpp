//
//  Geometry.cpp
//  FastXml
//
//  Created by z on 1/14/19.
//

#include "hsim/Geometry.hpp"

namespace hsim {

Matrix3 MassProperties::ComputeInertia(const Geometry& geometry, Real mass)
{
  switch (geometry.Type()) {
    case kBox:
      return ComputeInertia(static_cast<const Box&>(geometry), mass);
    case kSphere:
      return ComputeInertia(static_cast<const Sphere&>(geometry), mass);
    case kCone:
      return ComputeInertia(static_cast<const Cone&>(geometry), mass);
  }
}

Matrix3 MassProperties::ComputeInertia(const Box& box, Real mass)
{
  Matrix3 inertia = Matrix3::Identity();
  Real width = box.Width();
  Real height = box.Height();
  Real depth = box.Depth();
  inertia(0, 0) = mass / 12.0 * (std::pow(height, 2) + std::pow(depth, 2));
  inertia(1, 1) = mass / 12.0 * (std::pow(width, 2) + std::pow(depth, 2));
  inertia(2, 2) = mass / 12.0 * (std::pow(width, 2) + std::pow(height, 2));
  return inertia;
}

Matrix3 MassProperties::ComputeInertia(const Sphere& sphere, Real mass)
{
  const Real radius = sphere.Radius();
  Matrix3 inertia = Matrix3::Identity() *
    2.0 / 5.0 * mass * radius * radius;
  return inertia;
}

Matrix3 MassProperties::ComputeInertia(const Cone& cone, Real mass)
{
  const Real radius = cone.Radius();
  const Real height = cone.Height();
  const Real radius_sq = radius * radius;
  const Real height_sq = height * height;
  const Real tipsy = 3.0 / 5.0 * mass * height_sq + 3.0 / 20.0 * mass * radius_sq;
  Matrix3 inertia = Matrix3::Zero();
  inertia(0, 0) = tipsy;
  inertia(1, 1) = 3.0 / 10.0 * mass * radius_sq;
  inertia(2, 2) = tipsy;
  return inertia;
}

Real MassProperties::ComputeVolume(const Geometry& geometry)
{
  switch (geometry.Type()) {
    case kBox:
      return ComputeVolume(static_cast<const Box&>(geometry));
    case kSphere:
      return ComputeVolume(static_cast<const Sphere&>(geometry));
    case kCone:
      return ComputeVolume(static_cast<const Cone&>(geometry));
  }
}

Real MassProperties::ComputeVolume(const Box& box)
{
  return box.Width() * box.Height() * box.Depth();
}

Real MassProperties::ComputeVolume(const Sphere& sphere)
{
  return M_PI * 4.0 / 3.0 * std::pow(sphere.Radius(), 3);
}

Real MassProperties::ComputeVolume(const Cone& cone)
{
  const Real radius = cone.Radius();
  const Real height = cone.Height();
  return M_PI * radius * radius * height / 3;
}

}
