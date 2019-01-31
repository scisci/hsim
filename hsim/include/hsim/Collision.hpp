//
//  Collision.hpp
//  hsimall
//
//  Created by z on 1/25/19.
//

#ifndef HSIM_COLLISION_HPP
#define HSIM_COLLISION_HPP

#include "hsim/Math.hpp"
#include "hsim/Geometry.hpp"
#include "hsim/PxEngine.hpp"
#include "hsim/MotionPath.hpp"

#include <random>

namespace hsim {

/*
class CollisionObject {
public:
  CollisionObject(const Geometry& geom, const Transform& transform)
  : px_geom_(PxFactory::CreateGeometry(geom))
    px_transform(PxFactory::CreateTransform(transform))
  {}
  
private:
  physx::PxGeometryHolder px_geom_;
  physx::PxTransform px_transform_;
};
*/


// TODO:
// 1. Create random point from sampler
// 2. Point is already valid, but could check validity again somehow
// 3. Add point to graph
// 4. Parabola from each existing point to this new point
// 5. If any parabolas connect, then add edge
// 6. Check to see if goal is connected

class StateValidator {
public:
  virtual ~StateValidator() {}
  virtual bool Validate(const Eigen::Ref<const Vector3>& state) const = 0;
};




class Environment {
public:
  typedef std::size_t ObjectID;
  
  struct Object {
    ObjectID id;
    physx::PxGeometryHolder px_geom;
    physx::PxTransform px_transform;
  };
  
  Environment();
  
  ObjectID AddObstacle(const Geometry& geom, const Transform& transform);
  
  const std::vector<const Object>& Objects() const
  {
    return objects_;
  }
  
private:
  ObjectID id_;
  std::vector<const Object> objects_;
};
class RaycastQuery {
public:
  RaycastQuery(const Environment& environment);
  
  
  struct Result {
    Environment::ObjectID id;
    Vector3 position;
  };
  
  
  std::vector<Result> Query(const Ray& ray) const;

private:
  
  
  struct Hit {
    physx::PxRaycastHit hit;
    const Environment::Object *object;
  };
  
  
  //! Only used to store hits so we don't have to allocate memory each time (not thread safe)
  mutable std::vector<Hit> hits_;
  const Environment *environment_;
};

class Sampler {
public:
  virtual ~Sampler() {}
  virtual std::pair<Vector3, bool> Sample() = 0;
  virtual std::pair<Vector3, bool> SampleNear(const Eigen::Ref<const Vector3>& sample, Real dist) = 0;
};

class RaycastQuerySampler : public Sampler {
public:
  typedef Eigen::AlignedBox<Real, 3> SampleSpace;
  
  RaycastQuerySampler(
    const RaycastQuery& query,
    const SampleSpace& sample_space,
    Eigen::Index sample_axis,
    Real axis_dir,
    int64_t seed)
    : query_(&query),
      sample_space_(sample_space),
      sample_axis_(sample_axis),
      dir_(Vector3::Unit(sample_axis) * (axis_dir >= 0 ? 1 : -1)),
      ray_dist_(sample_space_.sizes()[sample_axis]),
      rng_(seed)
  {}
  
  virtual std::pair<Vector3, bool> Sample()
  {
    return SampleInternal(sample_space_, ray_dist_);
  }
  
  virtual std::pair<Vector3, bool> SampleNear(const Eigen::Ref<const Vector3>& sample, Real dist)
  {
    // Intersect the sample space
    Vector3 offset(dist, dist, dist);
    SampleSpace neighborhood = sample_space_.intersection(
      SampleSpace(sample - offset, sample + offset));
    Real ray_dist = neighborhood.sizes()[sample_axis_];
    
    return SampleInternal(neighborhood, ray_dist);
  }
  
private:
  std::pair<Vector3, bool> SampleInternal(
    const SampleSpace& sample_space,
    Real ray_dist)
  {
    const int max_attempts = 5;
    for (int i = 0; i < max_attempts; ++i) {
      Vector3 point = sample_space.sample();

      point(sample_axis_) = dir_(sample_axis_) >= 0 ?
        sample_space.min()[sample_axis_] : sample_space.max()[sample_axis_];
      
      auto results = query_->Query(Ray(point, point + dir_ * ray_dist));

      // Did it hit the floor? We need to check if there are no results and
      // the sample space intersects the floor
      if (results.empty()) {
        if (SampleSpaceIncludesFloor(sample_space)) {
          // 33% chance of hitting floor
          std::uniform_int_distribution<std::size_t> uniform(0, 2);
          if (uniform(rng_) == 0) {
            point(sample_axis_) += dir_(sample_axis_) * ray_dist;
            return std::make_pair(point, true);
          }
        }
        
        continue;
      }
      
      std::uniform_int_distribution<std::size_t> uniform(0, results.size() - 1);
      return std::make_pair(results[uniform(rng_)].position, true);
    }
    
    return std::make_pair(Vector3::Zero(), false);
  }
  
  inline bool SampleSpaceIncludesFloor(const SampleSpace& sample_space) const
  {
    if (&sample_space == &sample_space_) {
      return true;
    }
    
    if (dir_(sample_axis_) >= 0 &&
      sample_space.max()[sample_axis_] == sample_space_.max()[sample_axis_]) {
      return true;
    }
    
    if (dir_(sample_axis_) < 0 &&
      sample_space.min()[sample_axis_] == sample_space_.min()[sample_axis_]) {
      return true;
    }
    
    return false;
  }
  
  const RaycastQuery* query_;
  SampleSpace sample_space_;
  Eigen::Index sample_axis_;
  Vector3 dir_;
  Real ray_dist_;
  std::mt19937_64 rng_;
  
};



class DiscreteMotionPathCollisionDetector : public MotionPathCollisionDetector {
public:
  DiscreteMotionPathCollisionDetector(const Environment& environment, const RigidBody& robot, Real max_step_size)
  : contact_offset_(0.001),
    max_step_size_(max_step_size),
    environment_(&environment),
    robot_(&robot)
  {
    if (!robot.Shapes().empty()) {
      const auto& shape = robot.Shapes()[0];
      
      // Inset the geometry by a contact offset so that we don't get overlaps
      // if its almost touching
      auto collision_geom = shape->Geometry().Expand(-contact_offset_);
      
      robot_geom_ = PxFactory::CreateGeometry(*collision_geom.get());
      robot_tf_ = PxFactory::CreateTransform(shape->Transform());
      
      // Make sure foot is aligned to path
      auto aabb = shape->Geometry().BoundingBox();
      robot_shift_ = physx::PxVec3(0, aabb.sizes()[UpIdx] / 2, 0);
    } else {
      assert(0);
    }
  }
  
  bool CheckCollision(const Eigen::Ref<const Vector3>& pos) const
  {
    physx::PxTransform tf = robot_tf_;
    tf.p += robot_shift_ + physx::PxVec3(pos.x(), pos.y(), pos.z());
    
    const std::vector<const Environment::Object> objects = environment_->Objects();
    for (auto it = objects.begin(); it != objects.end(); ++it) {
      if (physx::PxGeometryQuery::overlap(
          robot_geom_.any(), tf, it->px_geom.any(), it->px_transform)) {
        return true;
      }
    }
    return false;
  }
 
  bool CheckCollision(const MotionPath& path) const
  {
    const Real length = path.Length();
    const size_t num_steps = ceil(length / max_step_size_);
    const Real step_size = length / num_steps;
    
    // Rotate robot along path?
    physx::PxTransform tf = robot_tf_;
    const physx::PxVec3 p = tf.p + robot_shift_;
    
    const std::vector<const Environment::Object> objects = environment_->Objects();
    for (auto it = objects.begin(); it != objects.end(); ++it) {
      for (size_t i = 1; i <= num_steps - 1; ++i) {
        Vector3 pos = path.Compute(i * step_size);
        tf.p = p + physx::PxVec3(pos.x(), pos.y(), pos.z());
        if (physx::PxGeometryQuery::overlap(
          robot_geom_.any(), tf, it->px_geom.any(), it->px_transform)) {
          return true;
        }
      }
    }
    
    return false;
  }
  
private:
  Real contact_offset_;
  Real max_step_size_;
  const Environment *environment_;
  const RigidBody *robot_;
  physx::PxGeometryHolder robot_geom_;
  physx::PxTransform robot_tf_;
  physx::PxVec3 robot_shift_;
};


class CollisionStateValidator : public StateValidator {
public:
  CollisionStateValidator(const MotionPathCollisionDetector& collision)
  : collision_(&collision)
  {}
  
  virtual bool Validate(const Eigen::Ref<const Vector3>& state) const
  {
    return !collision_->CheckCollision(state);
  }
  
private:
  const MotionPathCollisionDetector *collision_;
};

} // namespace hsim

#endif /* HSIM_COLLISION_HPP */
