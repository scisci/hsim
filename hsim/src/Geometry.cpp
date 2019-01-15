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
  Matrix3 inertia = Matrix3::Identity();
  inertia(0, 0) = 2.0 / 5.0 * mass * std::pow(sphere.Radius(), 2);
  inertia(1, 1) = inertia(0, 0);
  inertia(2, 2) = inertia(0, 0);
  return inertia;
}

Real MassProperties::ComputeVolume(const Geometry& geometry)
{
  switch (geometry.Type()) {
    case kBox:
      return ComputeVolume(static_cast<const Box&>(geometry));
    case kSphere:
      return ComputeVolume(static_cast<const Sphere&>(geometry));
  }
}

Real MassProperties::ComputeVolume(const Box &box)
{
  return box.Width() * box.Height() * box.Depth();
}

Real MassProperties::ComputeVolume(const Sphere &sphere)
{
  return M_PI * 4.0 / 3.0 * std::pow(sphere.Radius(), 3);
}

}
