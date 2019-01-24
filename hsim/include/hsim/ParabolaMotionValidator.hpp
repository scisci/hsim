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
    depth_index_(2), // z depth
    max_height_(100.0)
  {}
  
  Vector4 ComputeCoefficients(
    const Eigen::Ref<Vector3>& p0,
    const Eigen::Ref<Vector3>& n0,
    const Eigen::Ref<Vector3>& p1,
    const Eigen::Ref<Vector3>& n1) const
  {
    const Vector3 dif = p1 - p0;
    // Rotate points along up axis onto 2d plane defined by theta
    const Real theta = atan2(dif(depth_index_), dif(0));
    const Real x_theta0 = cos(theta) * p0(0) + sin(theta) * p0(depth_index_);
    const Real x_theta1 = cos(theta) * p1(0) + sin(theta) * p1(depth_index_);
    const Real width = cos(theta) * dif(0) + sin(theta) * dif(depth_index_);
    const Real up_offset = dif(up_index_);
    const Real phi = atan(friction_); // phi is angle of repose
    
    // 5th Constraint
    Real delta0;
    Real delta1;
    bool ok = false;
    //auto c0_delta = std::make_pair(0.0, false);
    //auto c1_delta = std::make_pair(0.0, false);
    if (!IsNormalVertical(n0)) {
      std::tie(delta0, ok) = TestConePlaneIntersection(p0, n0, friction_, theta, 1);
      if (!ok) {
        // Return empty path
        assert(0);
      }
    } else {
      delta0 = phi;
    }
    
    if (!IsNormalVertical(n1)) {
      std::tie(delta0, ok) = TestConePlaneIntersection(p1, n1, friction_, theta, 2);
      if (!ok) {
        // Return empty path
        assert(0);
      }
    } else {
      delta1 = phi;
    }
    
    // Gamma theta angles
    const Real n0_angle = atan2(n0(up_index_), cos(theta) * n0(0) +
      sin(theta) * n1(depth_index_));
    const Real n1_angle = atan2(n1(up_index_), cos(theta) * n1(0) +
      sin(theta) * n1(depth_index_));
    
    const Real alpha_0_min = n0_angle - delta0;
    const Real alpha_0_max = n0_angle + delta0;
    
    const Real alpha_inf4 = atan(dif(up_index_) / width);
    
    Real alpha_imp_min = n1_angle - M_PI - delta1;
    Real alpha_imp_max = n1_angle + M_PI + delta1;
    
    if (n1_angle < 0) {
      alpha_imp_min = n1_angle + M_PI - delta1;
      alpha_imp_max = n1_angle + M_PI + delta1;
    }
    
    Real alpha_lim_plus;
    Real alpha_lim_minus;
    std::tie(alpha_lim_plus, alpha_lim_minus, ok) = TestSecondConstraint(width, dif(up_index_));
    if (!ok) {
      // Return empty path
      assert(0);
    }
    
    Real alpha_imp_plus;
    Real alpha_imp_minus;
    std::tie(alpha_imp_plus, alpha_imp_minus, ok) = TestSixthConstraint(width, dif(up_index_));
    if (!ok) {
      // return empty path
      assert(0);
    }
    
    Real alpha_imp_inf;
    Real alpha_imp_sup;
    std::tie(alpha_imp_inf, alpha_imp_sup, ok) = TestThirdConstraint(width, dif(up_index_), alpha_imp_min, alpha_imp_max, n1_angle);
    if (!ok) {
      // return empty path
      assert(0);
    }
    
    Real alpha_inf_bound = 0;
    Real alpha_sup_bound = 0;
    
    /* Define alpha_0 interval satisfying constraints */
    if (n1_angle > 0) {
      alpha_lim_minus = std::max(alpha_lim_minus, alpha_imp_minus);
      alpha_inf_bound = std::max(std::max(alpha_imp_inf, alpha_lim_minus),
        std::max(alpha_0_min, alpha_inf4 + alpha_));
      
      if (alpha_imp_min < -M_PI / 2) {
        alpha_lim_plus = std::min(alpha_lim_plus, alpha_imp_minus);
        alpha_sup_bound = std::min(alpha_0_max,
          std::min(alpha_lim_plus, M_PI / 2));
      } else {// alpha_imp_sup is worth
        alpha_lim_plus = std::min(alpha_lim_plus, alpha_imp_plus);
        alpha_sup_bound = std::min(std::min(alpha_0_max, M_PI / 2),
          std::min(alpha_lim_plus, alpha_imp_sup));
      }
    } else { // down oriented cone
      if (alpha_imp_max < M_PI / 2) {
        alpha_lim_minus = std::max(alpha_lim_minus, alpha_imp_minus);
        alpha_inf_bound = std::max(std::max(alpha_imp_inf, alpha_lim_minus),
          std::max(alpha_0_min, alpha_inf4 + alpha_));
      } else {// alpha_imp_max >= M_PI/2 so alpha_imp_inf inaccurate
        alpha_lim_minus = std::max(alpha_lim_minus, alpha_imp_minus);
        alpha_inf_bound = std::max(std::max(alpha_0_min, alpha_inf4 + alpha_),
          alpha_lim_minus);
      }
      
      alpha_lim_plus = std::min(alpha_lim_plus, alpha_imp_plus);
      alpha_sup_bound = std::min(std::min(alpha_0_max, M_PI / 2),
        std::min(alpha_lim_plus, alpha_imp_sup));
    }
    
    if (alpha_inf_bound > alpha_sup_bound) {
      assert(0);
      // constraints intersection is empty
      // return empty
    }
    
    Real alpha = 0.5 * (alpha_inf_bound + alpha_sup_bound);
    auto coefs = ComputeCoefficients(alpha, theta, width, up_offset, x_theta0, p0(up_index_));
    auto coefs_inf = ComputeCoefficients(alpha_inf_bound, theta, width, up_offset, x_theta0, p0(up_index_));
    bool max_height_respected = IsMaxHeightRespected(coefs_inf, x_theta0, x_theta1);
    if (!max_height_respected) {
      assert(0);
      // Path is out of bounds
    }
    
    // not sure why
    max_height_respected = IsMaxHeightRespected(coefs, x_theta0, x_theta1);
    
    // TODO: dichotomy now
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
  
  /*!
    This uses the arc length formula:
    https://www.mathsisfun.com/calculus/arc-length.html
    So we integrate on sqrt(1 + f'(x)^2)
  */
  Real ComputeLength(
    const Eigen::Ref<Vector3>& p0,
    const Eigen::Ref<Vector3>& p1,
    const Eigen::Ref<Vector4>& coefs) const
  {
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
    // for N = 4, computation error ~= 1e-5.
    // for N = 20, computation error ~= 1e-11.
    const Real dx = (x2 - x1) / n;
    for (int i = 0; i < n; ++i) {
      // Here Simpson's rule (I think) is used to combine the trapezoidal
      // approximation and the midpoint approximation (T + 2M)/3
      length += dx * (
        ArcLength(x1 + i * dx, coefs) / 6.0 + // start * 1/6
        2.0 * ArcLength(x1 + (i + 0.5) * dx, coefs) / 3.0 + // mid * 2/3
        ArcLength(x1 + (i + 1) * dx, coefs) / 6.0); // end * 1/6
    }
    
    return length;
  }
  
  Vector3 ParabolaParam(
    Real param,
    const Eigen::Ref<Vector3>& p0,
    const Eigen::Ref<Vector3>& p1,
    Real length,
    const Eigen::Ref<Vector4>& coefs)
  {
    Vector3 result;
    const Real u = param / length;
    const Real theta = coefs(3);
    const Real x_theta_max = -0.5 * coefs(1) / coefs(0);
    const Real x_theta_initial =
      cos(theta) * p0(0) +
      sin(theta) * p0(depth_index_);
    const Real x_theta_end =
      cos(theta) * p1(0) +
      sin(theta) * p1(depth_index_);
    const Real u_max =
      (x_theta_max - x_theta_initial) /
      (x_theta_end - x_theta_initial);
    
    const bool tan_undefined =
      (theta < M_PI / 2 + 1e-2 && theta > M_PI / 2 - 1e-2) ||
      (theta > -M_PI / 2 - 1e-2 && theta < -M_PI/2 + 1e-2);
    
    if (!tan_undefined) {
      Real tan_theta = tan(theta);
      result(0) = (1 - u) * p0(0) + u * p1(0);
      result(depth_index_) = tan_theta * result(0) - tan_theta * p0(0) + p0(depth_index_);
      Real x_theta = cos(theta) * result(0) + sin(theta) * result(depth_index_);
      result(up_index_) = coefs(0) * x_theta * x_theta + coefs(1) * x_theta + coefs(2);
    } else {
      result(0) = p0(0);
      result(depth_index_) = (1 - u) * p0(depth_index_) + u * p1(depth_index_);
      Real x_theta = cos(theta) * result(0) + sin(theta) * result(depth_index_);
      result(up_index_) = coefs(0) * x_theta * x_theta + coefs(1) * x_theta + coefs(2);
    }
    
    return result;
  }
  
private:
  // Function equivalent to sqrt( 1 + f'(x)^2 )
  inline Real ArcLength(Real x, const Vector4& coefs) const
  {
    const Real v = 2 * coefs(0) * x + coefs(1);
    return sqrt(1.0 + v * v);
  }
  
  inline bool IsNormalVertical(const Eigen::Ref<Vector3>& n) const
  {
    // This calculation doesn't appear to be based on any particular metric. It
    // should return true for any very vertical cone which allows it to skip
    // one of the constraint checks.
    return 1000.0 * (n(0) * n(0) + n(depth_index_) * n(depth_index_)) <=
      n(up_index_) * n(up_index_);
  }
  
  bool IsMaxHeightRespected(
    const Eigen::Ref<Vector4>& coefs,
    Real x_theta0,
    Real x_theta_imp) const
  {
    const Real x_theta_max = -0.5 * coefs(1) / coefs(0);
    const Real z_x_theta_max = coefs(0) * x_theta_max * x_theta_max +
      coefs(1) * x_theta_max + coefs(2);
    if (x_theta0 <= x_theta_max && x_theta_max <= x_theta_imp) {
      if (z_x_theta_max > max_height_) {
        return false;
      }
    }
    
    return true;
  }
  
  std::tuple<Real, Real, bool> TestSecondConstraint(Real width, Real up_offset) const
  {
    const Real a = gravity_ * width * width;
    const Real b = -2 * width * vel0_max_ * vel0_max_;
    const Real c = gravity_ * width * width + 2 * up_offset * vel0_max_ * vel0_max_;
    const Real delta = b * b - 4 * a * c;
    
    if (delta < 0) {
      return std::make_tuple(0.0, 0.0, false);
    }
    
    if (width > 0) {
      return std::make_tuple(
        atan(0.5 * (-b + sqrt(delta)) / a),
        atan(0.5 * (-b - sqrt(delta)) / a), true);
    }
    
    return std::make_tuple(
      atan(0.5 * (-b + sqrt(delta)) / a) + M_PI,
      atan(0.5 * (-b - sqrt(delta)) / a) + M_PI, true);
  }
  
  std::tuple<Real, Real, bool> TestThirdConstraint(
    Real width,
    Real up_offset,
    Real alpha_imp_min,
    Real alpha_imp_max,
    Real n1_angle) const
  {
    if (width > 0) {
      if (n1_angle > 0) {
        if (alpha_imp_max > -M_PI / 2) {
          return std::make_tuple(
            atan(-tan(alpha_imp_min) + 2 * up_offset / width),
            atan(-tan(alpha_imp_max) + 2 * up_offset / width), true);
        }
        
        return std::make_tuple(0.0, 0.0, false);
      }
      
      if (alpha_imp_min < M_PI / 2) {
        return std::make_tuple(
          atan(-tan(alpha_imp_min) + 2 * up_offset / width),
          atan(-tan(alpha_imp_max) + 2 * up_offset / width), true);
      }
      
      return std::make_tuple(0.0, 0.0, false);
    }
    
    // X < 0   // TODO: cases n2_angle > 0 or < 0 (2D only)
    if (alpha_imp_min < -M_PI / 2) {
      return std::make_tuple(
        atan(-tan(alpha_imp_min) + 2 * up_offset / width) + M_PI,
        atan(-tan(alpha_imp_max) + 2 * up_offset / width) + M_PI, true);
    }
    
    return std::make_tuple(0.0, 0.0, false);
  }
  
  std::tuple<Real, Real, bool> TestSixthConstraint(Real width, Real up_offset) const
  {
    const Real a = gravity_ * width * width;
    const Real b = -2 * width * vel1_max_ * vel1_max_ -
      4 * width * up_offset * gravity_;
    const Real c = gravity_ * width * width +
      2 * up_offset * vel1_max_ * vel1_max_ +
      4 * gravity_ * up_offset * up_offset;
    const Real delta = b * b - 4 * a * c;
    
    if (delta < 0) {
      return std::make_tuple(0.0, 0.0, false);
    }
    
    if (width > 0) {
      return std::make_tuple(
        atan(0.5 * (-b + sqrt(delta)) / a),
        atan(0.5 * (-b - sqrt(delta)) / a), true);
    }
    
    return std::make_tuple(
      atan(0.5 * (-b + sqrt(delta)) / a) + M_PI,
      atan(0.5 * (-b - sqrt(delta)) / a) + M_PI, true);
  }
  
  std::pair<Real, bool> TestConePlaneIntersection(
    const Eigen::Ref<Vector3>& p, // point
    const Eigen::Ref<Vector3>& n, // normal
    Real mu, // coefficient of friction
    Real theta, // angle of vertical plane
    int num) const
  {
    const Real phi = atan(mu); // Coefficient of friction (half friction cone angle)
    const Real mu_sq = mu * mu;
    const Real u = n(0);
    const Real v = n(depth_index_);
    const Real w = n(up_index_);

    const Real denom_k = u * u + v * v - w * w * mu_sq;
    
    const bool tan_theta_def = theta != M_PI / 2 && theta != -M_PI / 2;

    // I guess here it is rotating the normal onto the plane and seeing its
    // angle on that plane
    const Real psi = M_PI / 2 - atan2(w, u * cos(theta) + v * sin(theta));
    const bool non_vert_cone =
      (psi < -phi && psi >= -M_PI / 2) ||
      (psi > phi && psi < M_PI - phi) ||
      (psi > M_PI + phi && psi <= 3 * M_PI / 2);
    const Real epsilon = !non_vert_cone && denom_k < 0 ? -1 : 1;
    if (denom_k > -1e-6 && denom_k < 1e-6) {
      if (tan_theta_def) {
        const Real tan_theta = tan(theta);
        if (u + v * tan_theta != 0) {
          const Real tan_theta_sq = tan_theta * tan_theta;
          
          const Real num_h = mu_sq * (u * u + v * v * tan_theta_sq) -
            u * u * tan_theta_sq - v * v + 2 * u * v * tan_theta * (1 + mu_sq) -
            w * w * (1 + tan_theta_sq);
          const Real h = -num_h /
            (2 * (1 + mu_sq) * fabs(w) * fabs(u + v * tan_theta));
          const Real cos2delta = h / sqrt(1 + tan_theta_sq * h * h);
          auto result = std::make_pair(0.5 * acos(cos2delta), true);
          assert(result.first <= phi + 1e-5);
          return result;
        } else { // // U + V*tanTheta = 0
          return std::make_pair(M_PI / 4, true);
        }
      } else { // theta = +- pi/2
        if (v != 0) {
          const Real l = -(v * v * (1 + mu_sq) - 1) /
            (2 * (1 + mu_sq) * fabs(v) * fabs(w));
          const Real cos2delta = l / sqrt(1 + l * l);
          auto result = std::make_pair(0.5 * acos(cos2delta), true);
          assert(result.first <= phi + 1e-5);
          return result;
        } else { // v = 0
          // cone plane intersection is a line
          return std::make_pair(0.0, false);
        }
      }
    }
    
    if (tan_theta_def) {
      Real x_plus, x_minus, z_x_plus, z_x_minus;
      const Real tan_theta = tan(theta);
      const Real tan_theta_sq = tan_theta * tan_theta;
      const Real discr = (u * u + w * w) * mu_sq - v * v -
        u * u * tan_theta_sq +
        (v * v + w * w) * mu_sq * tan_theta_sq +
        2 * (1 + mu_sq) * u * v * tan_theta;
      if (discr < 5e-2) { // cone-plane intersection too small or empty
        return std::make_pair(0.0, false);
      }
      const Real tmp = u * w + u * w * mu_sq + v * w * tan_theta +
        v * w * mu_sq * tan_theta;
      const Real k1 = (sqrt(discr) + tmp) / denom_k;
      const Real k2 = (-sqrt(discr) + tmp) / denom_k;
      
      if (non_vert_cone) {
        if (u * cos(theta) + v * sin(theta) < 0) {
          x_minus = -0.5;
        } else {
          x_minus = 0.5;
        }
        x_plus = x_minus;
        z_x_minus = x_minus * k2;
        z_x_plus = x_plus * k1;
        
        if (psi > M_PI / 2) { //  down: invert z_plus and z_minus
          z_x_plus = x_minus * k2;
          z_x_minus = x_plus * k1;
        }
      } else { // vertical cone
        if (-phi <= psi && psi <= phi) { // up
          x_minus = 0.5;
          if (denom_k < 0) {
            x_minus = 0.5;
            x_plus = -x_minus;
          } else {
            x_minus = -0.5;
            x_plus = x_minus;
          }
          z_x_minus = x_minus * k2;
          z_x_plus = x_plus * k1;
        } else { // vertical down
          if (denom_k < 0) {
            x_minus = -0.5;
            x_plus = -x_minus;
          } else {
            x_minus = 0.5;
            x_plus = x_minus;
          }
          z_x_minus = x_minus * k2;
          z_x_plus = x_plus * k1;
        }
      }
      
      Real cos2delta = epsilon * (1 + tan_theta_sq + k1 * k2) /
        sqrt((1 + tan_theta_sq + k1 * k1) * (1 + tan_theta_sq + k2 * k2));
      
      auto result = std::make_pair(0.5 * acos(cos2delta), true);
      assert(result.first <= phi + 1e-5);
      return result;
    } else { // theta = +- pi/2
      const Real discr = - u * u + (v * v + w * w) * mu_sq;
      if (discr < 5e-2) { // cone-plane intersection too small or empty
        return std::make_pair(0.0, false);
      }
      const Real g1 = ((1 + mu_sq) * v * w + sqrt(discr)) / denom_k;
      const Real g2 = ((1 + mu_sq) * v * w - sqrt(discr)) / denom_k;
      
      //Real y = 1;
      //if (theta == -M_PI / 2.0) {
      //  y = -1;
      //}
      //Real z_y_plus = g1 * y;
      //Real z_y_minus = g2 * y;
      const Real cos2delta = epsilon * (1 + g1 * g2) /
        sqrt((1 + g1 * g1) * (1 + g2 * g2));
      auto result = std::make_pair(0.5 * acos(cos2delta), true);
      assert(result.first <= phi + 1e-5);
      return result;
    }
  }

  Real gravity_;
  Real vel0_max_;
  Real vel1_max_;
  Real friction_;
  Real alpha_;
  Real max_height_;
  std::size_t search_limit_;
  const Eigen::Index up_index_;
  const Eigen::Index depth_index_;
};

} // namespace hsim

#endif /* HSIM_CONSTRAINED_JUMP_TRAJECTORY_H */
