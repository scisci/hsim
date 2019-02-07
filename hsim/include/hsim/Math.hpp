

#ifndef HSIM_MATH_HPP
#define HSIM_MATH_HPP

#include <Eigen/Core>

#include <iostream>

namespace hsim {

typedef double Real;

typedef Eigen::Matrix<Real, 3, 3> Matrix3;
typedef Eigen::Matrix<Real, 4, 4> Matrix4;
typedef Eigen::Matrix<Real, 2, 1> Vector2;
typedef Eigen::Matrix<Real, 3, 1> Vector3;
typedef Eigen::Matrix<Real, 4, 1> Vector4;
typedef Eigen::Matrix<Real, 6, 1> Vector6;
typedef Eigen::Transform<Real, 3, Eigen::Isometry> Transform;
typedef Eigen::Transform<Real, 3, Eigen::Affine> AffineTransform;
typedef Eigen::Transform<Real, 3, Eigen::Projective> ProjectionTransform;

typedef Eigen::AlignedBox<Real, 3> AlignedBox;
typedef Eigen::AngleAxis<Real> AngleAxis;
typedef Eigen::Translation<Real, 3> Translation;

constexpr Eigen::Index RtIdx = 0;
constexpr Eigen::Index UpIdx = 1;
constexpr Eigen::Index InIdx = 2;

typedef Eigen::Ref<const Vector2> Vector2Ref;
typedef Eigen::Ref<const Vector3> Vector3Ref;


enum class Handness {
  kLeft,
  kRight
};

void ToGLMatrix(const Transform& transform, float *out);
void ToGLMatrix(const Matrix4& transform, float *out);

AlignedBox TransformAlignedBox(
    const AlignedBox& geom_box,
    const Transform& t);

struct Ray {
  Ray()
  : start(0, 0, 0),
    end(0, 0, 0)
  {}
  
  Ray(const Eigen::Ref<const Vector3>& start, const Eigen::Ref<const Vector3>& end)
  : start(start),
    end(end)
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
  Real zfar,
  Handness handness=Handness::kRight);

Matrix4 CalcPerspectiveProjection(
  Real fovy,
  Real aspect,
  Real near,
  Real far,
  Handness handness=Handness::kRight);
  
Matrix4 CalcViewMatrix(
  const Vector3& eye,
  const Vector3& at,
  Handness handness=Handness::kRight);

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
