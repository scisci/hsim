//
//  ConstrainedJumpTrajectory.hpp
//  hsimall
//
//  Created by z on 1/22/19.
//

#ifndef HSIM_CONSTRAINED_JUMP_TRAJECTORY_H
#define HSIM_CONSTRAINED_JUMP_TRAJECTORY_H

namespace hsim {

class ParabolaMotionValidator {
public:
  ParabolaMotionValidator(Real vel_max, Real friction)
  : gravity_(9.81),
    vel0_max_(vel_max),
    vel1_max_(vel_max),
    friction_(friction),
    alpha_(0.001),
    search_limit_(7),
    up_index_(1), // y up
    depth_index_(2) // z depth
  {}
  
  Vector4 ComputeCoefficients(const Vector3& p0, const Vector3& n0, const Vector3& p1, const Vector3& n1) const
  {
    const Vector3 dif = p1 - p0;
    // Rotate points along up axis onto 2d plane defined by theta
    const Real theta = atan2(dif(depth_index_), dif(0));
    const Real x_theta0 = cos(theta) * p0(0) + sin(theta) * p0(depth_index_);
    const Real x_theta1 = cos(theta) * p1(0) + sin(theta) * p1(depth_index_);
    const Real width = cos(theta) * dif(0) + sin(theta) * dif(depth_index_);
    const Real up_offset = dif(up_index_);
    const Real phi = atan(friction_);
    
    return ComputeCoefficients(60 * M_PI / 180.0, theta, width, up_offset, x_theta0, p0(up_index_));
  }
  
  Vector4 ComputeCoefficients(
    Real angle,
    Real theta,
    Real width,
    Real up_offset,
    Real x,
    Real y) const
  {
    Vector4 coefs;
    const Real vel_x0 = sqrt((gravity_ * width * width) /
      (2.0 * (width * tan(angle) - up_offset)));
    const Real inv_vel_x0_sq = 1.0 / (vel_x0 * vel_x0);
    coefs(0) = -0.5 * gravity_ * inv_vel_x0_sq;
    coefs(1) = tan(angle) + gravity_ * x * inv_vel_x0_sq;
    coefs(2) = y - tan(angle) * x - 0.5 * gravity_ * x * x * inv_vel_x0_sq;
    coefs(3) = theta;
    return coefs;
  }
  
  Real ComputeLength(const Vector3& p0, const Vector3& p1, const Vector4& coefs) const
  {
    // Parabola is y = h * (1 - x^2 / a^2)
    // where h is height and a is half distance
    Real length = 0.0;
    Real x1 = p0(0);
    Real x2 = p1(0);
    Real theta = coefs(3);
    x1 = cos(theta) * p0(0) + sin(theta) * p0(depth_index_);
    x2 = cos(theta) * p1(0) + sin(theta) * p1(depth_index_);
    
    if (x1 > x2) {
      const Real tmp = x1;
      x1 = x2;
      x2 = tmp;
    }
    
    const int n = 6;
    const Real dx = (x2 - x1) / n;
    for (int i = 0; i < n; ++i) {
      length += dx * (
        DerivLength(x1 + i * dx, coefs) / 6.0 + // start * 1/6
        2.0 * DerivLength(x1 + (i + 0.5) * dx, coefs) / 3.0 + // mid * 2/3
        DerivLength(x1 + (i + 1) * dx, coefs) / 6.0); // end * 1/6
    }
    
    return length;
  }
  
  Vector3 ParabolaParam(Real param, const Vector3& start, const Vector3& end, Real length, const Vector4& coefs)
  {
    Vector3 result;
    const Real u = param / length;
    const Real theta = coefs(3);
    const Real x_theta_max = -0.5 * coefs(1) / coefs(0);
    const Real x_theta_initial =
      cos(theta) * start(0) +
      sin(theta) * start(depth_index_);
    const Real x_theta_end =
      cos(theta) * end(0) +
      sin(theta) * end(depth_index_);
    const Real u_max =
      (x_theta_max - x_theta_initial) /
      (x_theta_end - x_theta_initial);
    
    const bool tan_undefined =
      (theta < M_PI / 2 + 1e-2 && theta > M_PI / 2 - 1e-2) ||
      (theta > -M_PI / 2 - 1e-2 && theta < -M_PI/2 + 1e-2);
    
    if (!tan_undefined) {
      Real tan_theta = tan(theta);
      result(0) = (1 - u) * start(0) + u * end(0);
      result(depth_index_) = tan_theta * result(0) - tan_theta * start(0) + start(depth_index_);
      Real x_theta = cos(theta) * result(0) + sin(theta) * result(depth_index_);
      result(up_index_) = coefs(0) * x_theta * x_theta + coefs(1) * x_theta + coefs(2);
    } else {
      result(0) = start(0);
      result(depth_index_) = (1 - u) * start(depth_index_) + u * end(depth_index_);
      Real x_theta = cos(theta) * result(0) + sin(theta) * result(depth_index_);
      result(up_index_) = coefs(0) * x_theta * x_theta + coefs(1) * x_theta + coefs(2);
    }
    
    return result;
  }
  
private:
  // Function equivalent to sqrt( 1 + f'(x)^2 )
  Real DerivLength(Real x, const Vector4& coefs) const
  {
    const Real v = 2 * coefs(0) * x + coefs(1);
    return sqrt(1.0 + v * v);
  }

  Real gravity_;
  Real vel0_max_;
  Real vel1_max_;
  Real friction_;
  Real alpha_;
  std::size_t search_limit_;
  const Eigen::Index up_index_;
  const Eigen::Index depth_index_;
};

} // namespace hsim

#endif /* HSIM_CONSTRAINED_JUMP_TRAJECTORY_H */
