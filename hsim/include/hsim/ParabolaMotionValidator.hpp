//
//  ConstrainedJumpTrajectory.hpp
//  hsimall
//
//  Created by z on 1/22/19.
//

#ifndef HSIM_CONSTRAINED_JUMP_TRAJECTORY_H
#define HSIM_CONSTRAINED_JUMP_TRAJECTORY_H

#include "hsim/Math.hpp"
#include "hsim/RealRangeDichotomy.hpp"
#include "hsim/MotionPath.hpp"

namespace hsim {



class ParabolaPath : public MotionPath {
public:
  //! coefs(0)^2 + coefs(1) + coefs(2) where coefs(3) = theta
  typedef Eigen::Matrix<Real, 4, 1> Coefs;
  
  ParabolaPath(
    const Eigen::Ref<const Vector3>& start,
    const Eigen::Ref<const Vector3>& end,
    Real length,
    const Eigen::Ref<const Coefs>& coefs)
    : start_(start), end_(end), length_(length), coefs_(coefs)
    {}
  
  static Coefs ComputeCoefficients(
    //! Gravity in meters/sec
    Real gravity,
    //! Start angle of parabola in the plane
    Real alpha,
    //! The angle of the vertical parabola plane
    Real theta,
    //! The width of the parabola on the plane
    Real width,
    //! The difference in y on the plane from start to end
    Real up_offset,
    //! The starting x pos on plane
    Real x,
    //! The starting y pos on plane
    Real y)
  {
    ParabolaPath::Coefs coefs;
    const Real tan_alpha = tan(alpha);
    const Real vel_x0 = sqrt((gravity * width * width) /
      (2.0 * (width * tan_alpha - up_offset)));
    const Real inv_vel_x0_sq = 1.0 / (vel_x0 * vel_x0);
    
    coefs(0) = -0.5 * gravity * inv_vel_x0_sq;
    coefs(1) = tan_alpha + gravity * x * inv_vel_x0_sq;
    coefs(2) = y - tan_alpha * x - 0.5 * gravity * x * x * inv_vel_x0_sq;
    coefs(3) = theta;
    
    return coefs;
  }
  
  /*!
    This uses the arc length formula:
    https://www.mathsisfun.com/calculus/arc-length.html
    So we integrate on sqrt(1 + f'(x)^2)
  */
  static Real ComputeLength(
    const Eigen::Ref<const Vector3>& p0,
    const Eigen::Ref<const Vector3>& p1,
    const Eigen::Ref<const Coefs>& coefs)
  {
    const Real theta = coefs(3);
    Real length = 0.0;
    Real x1 = cos(theta) * p0(RtIdx) + sin(theta) * p0(InIdx);
    Real x2 = cos(theta) * p1(RtIdx) + sin(theta) * p1(InIdx);
    
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
  
  
  //! Param is curvlinear absisca (distance along path)
  static void ComputeParam(
    Real param,
    const Eigen::Ref<const Vector3>& p0,
    const Eigen::Ref<const Vector3>& p1,
    Real length,
    const Eigen::Ref<const Coefs>& coefs,
    Eigen::Ref<Vector3> out)
  {
    if (param == 0) {
      out = p0;
      return;
    }
    
    if (param == length) {
      out = p1;
      return;
    }

    const Real u = param / length;
    const Real theta = coefs(3);
    
    // Below is used for degree of freedom test not used
    
    //const Real x_theta_max = -0.5 * coefs(1) / coefs(0);
    //const Real x_theta_initial =
    //  cos(theta) * p0(RtIdx) +
    //  sin(theta) * p0(InIdx);
    //const Real x_theta_end =
    //  cos(theta) * p1(RtIdx) +
    //  sin(theta) * p1(InIdx);
    
    //const Real u_max =
    //  (x_theta_max - x_theta_initial) /
    //  (x_theta_end - x_theta_initial);
    
    const bool tan_undefined =
      (theta < M_PI / 2 + 1e-2 && theta > M_PI / 2 - 1e-2) ||
      (theta > -M_PI / 2 - 1e-2 && theta < -M_PI/2 + 1e-2);
    
    if (!tan_undefined) {
      const Real tan_theta = tan(theta);
      out(RtIdx) = (1 - u) * p0(RtIdx) + u * p1(RtIdx);
      out(InIdx) = tan_theta * out(RtIdx) - tan_theta * p0(RtIdx) + p0(InIdx);
      const Real x_theta = cos(theta) * out(RtIdx) + sin(theta) * out(InIdx);
      out(UpIdx) = coefs(0) * x_theta * x_theta + coefs(1) * x_theta + coefs(2);
      return;
    }
    
    out(RtIdx) = p0(RtIdx);
    out(InIdx) = (1 - u) * p0(InIdx) + u * p1(InIdx);
    const Real x_theta = cos(theta) * out(RtIdx) + sin(theta) * out(InIdx);
    out(UpIdx) = coefs(0) * x_theta * x_theta + coefs(1) * x_theta + coefs(2);
  }
  
  virtual Real Length() const
  {
    return length_;
  }
  
  virtual Vector3 Compute(Real param) const
  {
    Vector3 result;
    ComputeParam(param, start_, end_, length_, coefs_, result);
    return result;
  }
  
  void Compute(Real param, Eigen::Ref<Vector3> out) const
  {
    ComputeParam(param, start_, end_, length_, coefs_, out);
  }
  
private:
  // Function equivalent to sqrt( 1 + f'(x)^2 )
  static inline Real ArcLength(Real x, const Eigen::Ref<const Coefs>& coefs)
  {
    const Real v = 2 * coefs(0) * x + coefs(1);
    return sqrt(1.0 + v * v);
  }
  
  Vector3 start_;
  Vector3 end_;
  Real length_;
  Coefs coefs_;
};

class ParabolaMotionValidator {
public:
  ParabolaMotionValidator(
    const MotionPathCollisionDetector& collision,
    Real vel_max,
    Real friction)
    : gravity_(9.81),
      vel0_max_(vel_max),
      vel1_max_(vel_max),
      friction_(friction),
      alpha_(0.001),
      max_height_(100.0),
      search_limit_(7),
      collision_(&collision)
  {}
  
  std::unique_ptr<ParabolaPath> ComputePath(
    const Eigen::Ref<const Vector3>& p0,
    const Eigen::Ref<const Vector3>& n0,
    const Eigen::Ref<const Vector3>& p1,
    const Eigen::Ref<const Vector3>& n1) const
  {
    std::unique_ptr<ParabolaPath> result;
    
    const Vector3 dif = p1 - p0;
    // Rotate points along up axis onto 2d plane defined by theta
    const Real theta = atan2(dif(InIdx), dif(RtIdx));
    /*
    if (dif(InIdx) == 0.0 && dif(RtIdx) == 0) {
      std::cout << "jump in y only" << std::endl;
    }
    */
    const Real x_theta0 = cos(theta) * p0(RtIdx) + sin(theta) * p0(InIdx);
    const Real x_theta1 = cos(theta) * p1(RtIdx) + sin(theta) * p1(InIdx);
    const Real width = cos(theta) * dif(RtIdx) + sin(theta) * dif(InIdx);
    const Real up_offset = dif(UpIdx);
    const Real phi = ComputeHalfConeAngle(friction_); // phi is angle of repose, half friction cone
    
    // 5th Constraint
    Real delta0;
    Real delta1;
    bool ok = false;

    if (!IsNormalVertical(n0)) {
      std::tie(delta0, ok) = TestConePlaneIntersection(p0, n0, friction_, theta, 1);
      if (!ok) {
        return result;
      }
    } else {
      delta0 = phi;
    }
    
    if (!IsNormalVertical(n1)) {
      std::tie(delta1, ok) = TestConePlaneIntersection(p1, n1, friction_, theta, 2);
      if (!ok) {
        return result;
      }
    } else {
      delta1 = phi;
    }
    
    // Gamma theta angles
    const Real n0_angle = atan2(n0(UpIdx), cos(theta) * n0(RtIdx) +
      sin(theta) * n0(InIdx));
    const Real n1_angle = atan2(n1(UpIdx), cos(theta) * n1(RtIdx) +
      sin(theta) * n1(InIdx));
    
    const Real alpha_0_min = n0_angle - delta0;
    const Real alpha_0_max = n0_angle + delta0;
    
    // The angle from the start point to end point against x axis
    // alpha cannot be less than this obviously
    // In the case of a vertical jump its always 90 degrees
    const Real alpha_inf4 = width == 0.0 ? M_PI / 2 : atan(dif(UpIdx) / width);
    
    // Subtracting PI essentially inverts the cone which puts the normal
    // in line with the parabola trajectory
    Real alpha_imp_min = n1_angle - M_PI - delta1;
    Real alpha_imp_max = n1_angle - M_PI + delta1;
    
    if (n1_angle < 0) {
      alpha_imp_min = n1_angle + M_PI - delta1;
      alpha_imp_max = n1_angle + M_PI + delta1;
    }
    
    Real alpha_lim_minus;
    Real alpha_lim_plus;
    std::tie(alpha_lim_minus, alpha_lim_plus, ok) = TestSecondConstraint(width, dif(UpIdx));
    if (!ok) {
      return result;
    }
    
    Real alpha_imp_minus;
    Real alpha_imp_plus;
    std::tie(alpha_imp_minus, alpha_imp_plus, ok) = TestSixthConstraint(width, dif(UpIdx));
    if (!ok) {
      return result;
    }
    
    Real alpha_imp_inf;
    Real alpha_imp_sup;
    std::tie(alpha_imp_inf, alpha_imp_sup, ok) = TestThirdConstraint(width, dif(UpIdx), alpha_imp_min, alpha_imp_max, n1_angle);
    if (!ok) {
      return result;
    }
    
    Real alpha_inf_bound = 0;
    Real alpha_sup_bound = 0;
    
    /* Define alpha_0 interval satisfying constraints */
    if (n1_angle > 0) {
      alpha_lim_minus = std::max(alpha_lim_minus, alpha_imp_minus);
      alpha_lim_plus = std::min(alpha_lim_plus, alpha_imp_plus);
      
      alpha_inf_bound = std::max(
        {alpha_imp_inf, alpha_lim_minus, alpha_0_min, alpha_inf4 + alpha_});
      
      if (alpha_imp_min < -M_PI / 2) { // alpha_imp_sup is not defined
        alpha_sup_bound = std::min({alpha_0_max, alpha_lim_plus, M_PI / 2});
      } else { // alpha_imp_sup is defined
        alpha_sup_bound = std::min(
          {alpha_0_max, M_PI / 2, alpha_lim_plus, alpha_imp_sup});
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
    
    // TODO: a perfectly vertical jump fails here because the above alpha_inf_bound
    // has alpha_ added to it which will make it slightly larger than the
    // sup_bound
    if (alpha_inf_bound > alpha_sup_bound) {
      // constraints intersection is empty
      return result;
    }
    
    Real alpha = 0.5 * (alpha_inf_bound + alpha_sup_bound);
    auto coefs = ParabolaPath::ComputeCoefficients(
      gravity_, alpha, theta, width, up_offset, x_theta0, p0(UpIdx));
    auto coefs_inf = ParabolaPath::ComputeCoefficients(
      gravity_, alpha_inf_bound, theta, width, up_offset, x_theta0, p0(UpIdx));
    
    bool max_height_respected = IsMaxHeightRespected(coefs_inf, x_theta0, x_theta1);
    if (!max_height_respected) {
      // Path is out of bounds
      return result;
    }
    
    // not sure why
    max_height_respected = IsMaxHeightRespected(coefs, x_theta0, x_theta1);
    
    bool has_collisions = true;
    RealRangeDichotomy dichotomy(alpha_inf_bound, alpha_sup_bound, search_limit_);
    for (auto& alpha : dichotomy) {
      coefs = ParabolaPath::ComputeCoefficients(
        gravity_, alpha, theta, width, up_offset, x_theta0, p0(UpIdx));
      max_height_respected = IsMaxHeightRespected(coefs, x_theta0, x_theta1);
      const Real length = ParabolaPath::ComputeLength(p0, p1, coefs);
      std::unique_ptr<ParabolaPath> path(new ParabolaPath(p0, p1, length, coefs));
      
      // Do collision check
      has_collisions = collision_->CheckCollision(*path.get());

      if (!has_collisions && max_height_respected) {
        return path;
      }
    }

    return result;
  }
  
  static inline Real ComputeHalfConeAngle(Real friction)
  {
    return atan(friction);
  }
  
  
private:
  
  
  inline bool IsNormalVertical(const Eigen::Ref<const Vector3>& n) const
  {
    // This calculation doesn't appear to be based on any particular metric. It
    // should return true for any very vertical cone which allows it to skip
    // one of the constraint checks.
    return 1000.0 * (n(0) * n(0) + n(InIdx) * n(InIdx)) <=
      n(UpIdx) * n(UpIdx);
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
    // parabola dist = 2v^2 * (sin(alpha)cos(alpha)) / g
    // dist = 2v^2 * (.5 sin(2alpha)) / g
    // Since we know maximum is at alpha == 45deg then
    // dist = 2v^2 * .5 / g
    // dist = v^2 / g
    
    // This uses quadratic formula
    // x = (-b +- sqrt(b^2 - 4ac)) / 2a
    // to solve for tan(alpha)
    // if discriminant is < 0 there is no solution
    
    // If width is 0 it is just a vertical jump so only 2 possible angles
    if (width == 0.0) {
      return std::make_tuple(M_PI / 2, M_PI / 2, true);
    }
    
    const Real a = gravity_ * width * width;
    const Real b = -2 * width * vel0_max_ * vel0_max_;
    const Real c = gravity_ * width * width + 2 * up_offset * vel0_max_ * vel0_max_;
    const Real delta = b * b - 4 * a * c;
    
    if (delta < 0) {
      return std::make_tuple(0.0, 0.0, false);
    }
    
    if (width > 0) {
      return std::make_tuple(
        atan(0.5 * (-b - sqrt(delta)) / a),
        atan(0.5 * (-b + sqrt(delta)) / a), true);
    }
    
    return std::make_tuple(
      atan(0.5 * (-b - sqrt(delta)) / a) + M_PI,
      atan(0.5 * (-b + sqrt(delta)) / a) + M_PI, true);
  }
  
  std::tuple<Real, Real, bool> TestThirdConstraint(
    Real width,
    Real up_offset,
    Real alpha_imp_min,
    Real alpha_imp_max,
    Real n1_angle) const
  {
    // If width is 0 it is just a vertical jump so only 2 possible angles
    // TODO: should we check against alpha_imp in this case?
    if (width == 0.0) {
      return std::make_tuple(M_PI / 2, M_PI / 2, true);
    }
    
    if (width > 0) {
      if (n1_angle > 0) {
        if (alpha_imp_max > -M_PI / 2) {
          return std::make_tuple(
            atan(-tan(alpha_imp_max) + 2 * up_offset / width), // inf
            atan(-tan(alpha_imp_min) + 2 * up_offset / width), true); // sup
        }
        
        return std::make_tuple(0.0, 0.0, false);
      }
      
      if (alpha_imp_min < M_PI / 2) {
        return std::make_tuple(
          atan(-tan(alpha_imp_max) + 2 * up_offset / width), // inf
          atan(-tan(alpha_imp_min) + 2 * up_offset / width), true); // sup
      }
      
      return std::make_tuple(0.0, 0.0, false);
    }
    
    // X < 0   // TODO: cases n2_angle > 0 or < 0 (2D only)
    if (alpha_imp_min < -M_PI / 2) {
      return std::make_tuple(
        atan(-tan(alpha_imp_max) + 2 * up_offset / width) + M_PI, // inf
        atan(-tan(alpha_imp_min) + 2 * up_offset / width) + M_PI, true); //sup
    }
    
    return std::make_tuple(0.0, 0.0, false);
  }
  
  std::tuple<Real, Real, bool> TestSixthConstraint(Real width, Real up_offset) const
  {
    // If width is 0 it is just a vertical jump so only 2 possible angles
    if (width == 0.0) {
      return std::make_tuple(M_PI / 2, M_PI / 2, true);
    }
    
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
        atan(0.5 * (-b - sqrt(delta)) / a),
        atan(0.5 * (-b + sqrt(delta)) / a), true);
    }
    
    return std::make_tuple(
      atan(0.5 * (-b - sqrt(delta)) / a) + M_PI,
      atan(0.5 * (-b + sqrt(delta)) / a) + M_PI, true);
  }
  
  std::pair<Real, bool> TestConePlaneIntersection(
    const Eigen::Ref<const Vector3>& p, // point
    const Eigen::Ref<const Vector3>& n, // normal
    Real mu, // coefficient of friction
    Real theta, // angle of vertical plane
    int num) const
  {
    const Real phi = atan(mu); // Coefficient of friction (half friction cone angle)
    const Real mu_sq = mu * mu;
    const Real u = n(0);
    const Real v = n(InIdx);
    const Real w = n(UpIdx);

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
  const MotionPathCollisionDetector *collision_;
};

} // namespace hsim

#endif /* HSIM_CONSTRAINED_JUMP_TRAJECTORY_H */
