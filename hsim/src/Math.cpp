//
//  Math.cpp
//  FastXml
//
//  Created by z on 1/18/19.
//

#include "hsim/Math.hpp"
#include <Eigen/Geometry>

namespace hsim {

void ToGLMatrix(const Transform& transform, float *out)
{
  if (std::is_same<Real, float>::value) {
    memcpy(out, transform.data(), 16 * sizeof(float));
  }
  const Real *src = transform.data();
  for (int i = 0; i < 16; i++) {
    out[i] = src[i];
  }
}

AlignedBox TransformAlignedBox(const AlignedBox& geom_box, const Transform& t)
{
  AlignedBox box;
  box.extend(t * geom_box.corner(AlignedBox::CornerType::TopLeftFloor));
  box.extend(t * geom_box.corner(AlignedBox::CornerType::TopRightFloor));
  box.extend(t * geom_box.corner(AlignedBox::CornerType::BottomRightFloor));
  box.extend(t * geom_box.corner(AlignedBox::CornerType::BottomLeftFloor));
  box.extend(t * geom_box.corner(AlignedBox::CornerType::TopLeftCeil));
  box.extend(t * geom_box.corner(AlignedBox::CornerType::TopRightCeil));
  box.extend(t * geom_box.corner(AlignedBox::CornerType::BottomRightCeil));
  box.extend(t * geom_box.corner(AlignedBox::CornerType::BottomLeftCeil));
  return box;
}

Matrix4 CalcXYWHProjection(
  Real x,
  Real y,
  Real width,
  Real height,
  Real znear,
  Real zfar)
{
  const bool right_handed = true;
  
  const Real diff = zfar - znear;
  const Real aa = (zfar + znear) / diff;
  const Real bb = (2.0 * zfar * znear) / diff;
  
  Matrix4 result = Matrix4::Zero();
  result(0) = width;
  result(5) = height;
  result(8) = right_handed ? x : -x;
  result(9) = right_handed ? y : -y;
  result(10) = right_handed ? -aa : aa;
  result(11) = right_handed ? -1.0 : 1.0;
  result(14) = -bb;

  return result;
}

Matrix4 CalcPerspectiveProjection(
  Real fovy,
  Real aspect,
  Real near,
  Real far)
{
  const Real height = 1.0 / tanf(fovy * M_PI / 360.0);
  const Real width = height * 1.0 / aspect;
  return CalcXYWHProjection(0.0, 0.0, width, height, near, far);
}

Matrix4 CalcViewMatrix(const Vector3& eye, const Vector3& at)
{
  const bool right_handed = true;
  const Vector3 up(0.0, 1.0, 0.0);
  
  const Vector3 z_axis = (right_handed ? eye - at : at - eye).normalized();
  const Vector3 x_axis = up.cross(z_axis).normalized();
  const Vector3 y_axis = z_axis.cross(x_axis);

  Matrix4 orientation;
  orientation <<
    Vector4(x_axis.x(), y_axis.x(), z_axis.x(), 0.0),
    Vector4(x_axis.y(), y_axis.y(), z_axis.y(), 0.0),
    Vector4(x_axis.z(), y_axis.z(), z_axis.z(), 0.0),
    Vector4(-x_axis.dot(eye), -y_axis.dot(eye), -z_axis.dot(eye), 1);

  return orientation;
}

Vector4 NormalizeDeviceCoords(Real x, Real y, Real z, Real width, Real height)
{
  return Vector4((2.0 * x) / width - 1, 1.0 - (2.0 * y) / height, z, 1.0);
}

Ray CastRayFor2DCoords(
  Real x,
  Real y,
  Real width,
  Real height,
  const Matrix4& proj_matrix,
  const Matrix4& view_matrix,
  Real norm_start,
  Real norm_end)
{
  const Matrix4 mvpw = (proj_matrix * view_matrix).inverse();
  const Vector4 start = NormalizeDeviceCoords(x, y, norm_start, width, height);
  const Vector4 end = NormalizeDeviceCoords(x, y, norm_end, width, height);
  Vector4 ray_start = mvpw * start;
  Vector4 ray_end = mvpw * end;
  ray_start /= ray_start.w();
  ray_end /= ray_end.w();
  return Ray(ray_start.head<3>(), ray_end.head<3>());
}


} // namespace hsim
