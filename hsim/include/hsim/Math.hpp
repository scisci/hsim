

#ifndef HSIM_MATH_HPP
#define HSIM_MATH_HPP

#include <Eigen/Core>

#include <iostream>

namespace hsim {

typedef double Real;

typedef Eigen::Matrix<Real, 3, 3> Matrix3;
typedef Eigen::Matrix<Real, 4, 4> Matrix4;
typedef Eigen::Matrix<Real, 3, 1> Vector3;
typedef Eigen::Matrix<Real, 4, 1> Vector4;
typedef Eigen::Matrix<Real, 6, 1> Vector6;
typedef Eigen::Transform<Real, 3, Eigen::Isometry> Transform;
typedef Eigen::Transform<Real, 3, Eigen::Affine> AffineTransform;
typedef Eigen::Transform<Real, 3, Eigen::Projective> ProjectionTransform;

constexpr Eigen::Index RtIdx = 0;
constexpr Eigen::Index UpIdx = 1;
constexpr Eigen::Index InIdx = 2;

void ToGLMatrix(const Transform& transform, float *out);

struct Ray {
  Ray(const Vector3& start, const Vector3& end)
  : start(start),
    end(end)
  {}
  
  Ray(const Vector4& start, const Vector4& end)
  : start(start.x(), start.y(), start.z()),
    end(end.x(), end.y(), end.z())
  {}
  
  Vector3 PointAtDistance(Real dist)
  {
    return start + (end - start).normalized() * dist;
  }
  
  Vector3 start;
  Vector3 end;
};

Matrix4 CalcXYWHProjection(
  Real x,
  Real y,
  Real width,
  Real height,
  Real znear,
  Real zfar);

Matrix4 CalcPerspectiveProjection(Real fovy, Real aspect, Real near, Real far);
Matrix4 CalcViewMatrix(const Vector3& eye, const Vector3& at);

Vector4 NormalizeDeviceCoords(
  Real x,
  Real y,
  Real z,
  Real width,
  Real height);
  
Ray CastRayFor2DCoords(
  Real x,
  Real y,
  Real width,
  Real height,
  const Matrix4& projection_matrix,
  const Matrix4& view_matrix,
  Real start=-1.0,
  Real end=1.0);

} // namespace hsim

#endif // HSIM_MATH_HPP
