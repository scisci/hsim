//
//  JumpSolver.hpp
//  hsimall
//
//  Created by z on 2/4/19.
//

#ifndef HSIM_JUMP_SOLVER_HPP
#define HSIM_JUMP_SOLVER_HPP

#include "hsim/Math.hpp"
#include "hsim/Physics.hpp"
#include "hsim/ParabolaMotionValidator.hpp"
#include "hsim/Collision.hpp"
#include "hsim/ParabolicPlanner.hpp"


#include <unordered_map>

namespace hsim {

class JumpSolver {
public:
  JumpSolver()
  :robot_radius_(0.125),
   robot_density_(1000.0),
   max_jump_height_(1.5),
   friction_(1.2),
   goal_height_(1.5),
   goal_min_dist_(0.75),
   border_width_(0.25),
   path_step_size_(0.05),
   robot_(RigidBodyBuilder::Build(
    std::unique_ptr<Geometry>(new Sphere(robot_radius_)),
    robot_density_,
    Transform::Identity())),
   raycast_query_(enviro_),
   sampler_(raycast_query_, AlignedBox(), UpIdx, -1, 1),
   collider_(enviro_, *robot_.get(), path_step_size_),
   steerer_(collider_, sqrt(max_jump_height_ * 2 * 9.81), friction_),
   validator_(collider_),
   planner_(&sampler_, steerer_, validator_)
  {}
  
  void Solve(const RigidBody& body)
  {
    // We reuse these structures so we need to clear them
    enviro_.Clear();
    goal_obs_.clear();
    
    AddObstacles(body);
    
    AlignedBox sample_space = SelectSampleSpace(body);
    
    sampler_.SetSampleSpace(sample_space);
    std::vector<Vector3> start_points = SelectStartPoints(sample_space);
    std::vector<Vector3> goal_points = SelectGoalPoints(sample_space);
    
    planner_.Solve(start_points, goal_points);

  }
  
  std::vector<Vector3> Vertices() const
  {
    return planner_.Vertices();
  }
  
  std::vector<const MotionPath*> Edges() const
  {
    return planner_.Edges();
  }
  
private:
  inline Real GridSize() const
  {
    return robot_radius_ * 2;
  }
  
  std::vector<Vector3> SelectStartPoints(const AlignedBox& sample_space) const
  {
    return {sample_space.min() + Vector3(0.01, 0, 0.01)};
  }
  
  std::vector<Vector3> SelectGoalPoints(const AlignedBox& sample_space) const
  {
    std::vector<Vector3> all_goals;
    const Real grid_size = GridSize();
    
    // We can use the raycast query to find goals, each obstacle that is in
    // the goal range we can attempt to find a point on it that is valid
    // if so, its a goal point.
    for (auto& goal_entry : goal_obs_) {
      const Shape *shape = goal_entry.second;
      Transform transform = robot_->Transform() * shape->Transform();
      
      // Keeps track of how many goals were added for this obstacle
      std::size_t num_obs_goals = 0;
      
      // Each goal for the obstacle should be some distance from any other goal
      // just so we don't have too many goals, so we are effectively sweeping
      // the object along a grid, if we find a point that is far from any
      // previous point, we keep it.
      const Real min_dist_sq = grid_size * 3 * grid_size * 3;

      const AlignedBox bbox = TransformAlignedBox(
        shape->Geometry().BoundingBox(), transform);
      const auto sizes = bbox.sizes();
      const std::size_t nx = sizes[RtIdx] / grid_size;
      const std::size_t ny = sizes[InIdx] / grid_size;
      
      
      Ray ray;
      for (size_t x = 0; x < nx; ++x) {
        for (std::size_t y = 0; y < ny; ++y) {
          Vector3 start = bbox.center();
          start[RtIdx] += x * grid_size + grid_size / 2 - sizes[RtIdx] / 2;
          start[InIdx] += y * grid_size + grid_size / 2 - sizes[InIdx] / 2;
          start[UpIdx] = sample_space.max()[UpIdx];
          
          ray.start = start;
          ray.end = start - Vector3::Unit(UpIdx) * (start[UpIdx] - goal_height_);
          
          auto results = raycast_query_.Query(ray);
          for (auto& result : results) {
            // Check to see if the object we hit is the one we are testing,
            // ignore others.
            if (result.id != goal_entry.first) {
              continue;
            }

            bool too_close = false;
            // Check previous points to make sure this point isn't too close
            for (size_t i = all_goals.size() - num_obs_goals; i < all_goals.size(); ++i) {
              if ((all_goals[i] - result.position).squaredNorm() < min_dist_sq) {
                too_close = true;
                break;
              }
            }
            
            // Its a successful hit, lets do a penetration test
            if (!too_close && !collider_.CheckCollision(result.position)) {
              all_goals.push_back(result.position);
              num_obs_goals++;
            }
            
            break;
          }
        }
      }
    }
    
    return all_goals;
  }
  
  void AddObstacles(const RigidBody& body)
  {
    const Real grid_size = GridSize();
    
    // Find the bounding box of the new space
    for (auto& shape : body.Shapes()) {
      Transform transform = body.Transform() * shape->Transform();
      AlignedBox bbox = TransformAlignedBox(
        shape->Geometry().BoundingBox(), transform);
      
      Environment::ObjectID id = enviro_.AddObstacle(
        shape->Geometry(), transform);
      if (bbox.max()(UpIdx) >= goal_height_) {
        auto sizes = bbox.sizes();
        // Surface should be big enough to fit the object
        if (sizes[RtIdx] >= grid_size && sizes[InIdx] >= grid_size) {
          goal_obs_.insert(std::make_pair(id, shape.get()));
        }
      }
    }
  }
  
  AlignedBox SelectSampleSpace(const RigidBody& body) const
  {
    AlignedBox sample_space = RigidActor::BoundingBox(body, body.Transform());

    // Expand the box slightly to give starting space around it, we expand
    // around the sides, but also around the top to give the ray some distance
    sample_space.min() -= Vector3(border_width_, 0.0, border_width_);
    sample_space.max() += Vector3(border_width_, border_width_, border_width_);
    
    return sample_space;
  }
  
  //! The radius of the sphere representing the jumper
  Real robot_radius_;
  //! The density of the sphere representing the jumper (not used)
  Real robot_density_;
  //! The maximum height the robot can jump
  Real max_jump_height_;
  //! The friction of the jump surface
  Real friction_;
  
  //! The minimum height qualifier for a platform to be considered a goal
  Real goal_height_;
  //! The minimum distance between points on a platform to be added as a goal
  Real goal_min_dist_;
  
  //! The border around the rigid body for the floor
  Real border_width_;
  
  //! The integration step size of the discrete path collision detector
  Real path_step_size_;
  
  Environment enviro_;
  std::unique_ptr<RigidBody> robot_;
  std::unordered_map<Environment::ObjectID, const Shape*> goal_obs_;
  RaycastQuery raycast_query_;
  RaycastQuerySampler sampler_;
  DiscreteMotionPathCollisionDetector collider_;
  ParabolaMotionValidator steerer_;
  CollisionStateValidator validator_;
  ParabolicPlanner planner_;
};

} // namespace hsim

#endif /* HSIM_JUMP_SOLVER_HPP */
