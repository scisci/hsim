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

class RaycastQuery {
public:
  RaycastQuery();
  
  typedef std::size_t ObjectID;
  
  struct Result {
    ObjectID id;
    Vector3 position;
  };
  
  ObjectID Add(const Geometry& geom, const Transform& transform);
  
  std::vector<Result> Query(const Ray& ray) const;

private:
  struct Object {
    ObjectID id;
    physx::PxGeometryHolder px_geom;
    physx::PxTransform px_transform;
  };
  
  struct Hit {
    physx::PxRaycastHit hit;
    const Object *object;
  };
  
  ObjectID id_;
  std::vector<Object> objects_;
  
  //! Only used to store hits so we don't have to allocate memory each time (not thread safe)
  mutable std::vector<Hit> hits_;
};

class RaycastQuerySampler {
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
  
  std::pair<Vector3, bool> Sample()
  {
    return SampleInternal(sample_space_, ray_dist_);
  }
  
  std::pair<Vector3, bool> SampleNear(const Eigen::Ref<const Vector3>& sample, Real dist)
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

} // namespace hsim

#endif /* HSIM_COLLISION_HPP */
