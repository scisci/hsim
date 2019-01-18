

#ifndef HSIM_MATH_HPP
#define HSIM_MATH_HPP

#include <Eigen/Core>

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

} // namespace hsim

#endif // HSIM_MATH_HPP
