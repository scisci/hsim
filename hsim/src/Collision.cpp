//
//  Collision.cpp
//  hsim
//
//  Created by z on 1/25/19.
//

#include "hsim/Collision.hpp"
#include "PxPhysicsAPI.h"

namespace hsim {


Environment::Environment()
:id_(0)
{}

Environment::ObjectID Environment::AddObstacle(const Geometry& geom, const Transform& transform)
{
  objects_.push_back({
    .id = ++id_,
    .px_geom = PxFactory::CreateGeometry(geom),
    .px_transform = PxFactory::CreateTransform(transform)
  });
  
  return id_;
}
  

RaycastQuery::RaycastQuery(const Environment& environment)
: environment_(&environment)
{
}



std::vector<RaycastQuery::Result> RaycastQuery::Query(const Ray& ray) const
{
  std::vector<RaycastQuery::Result> results;
  
  Vector3 dir = ray.end - ray.start;
  physx::PxReal max_dist = dir.norm();
  dir.normalize();
  
  physx::PxVec3 origin(ray.start.x(), ray.start.y(), ray.start.z());
  physx::PxVec3 unit_dir(dir.x(), dir.y(), dir.z());
  
  physx::PxRaycastHit hit;
  
  hits_.clear();
  const std::vector<const Environment::Object> objects = environment_->Objects();
  for (auto it = objects.begin(); it != objects.end(); ++it) {
    const Environment::Object& object = *it;
    
    std::size_t num_hits = physx::PxGeometryQuery::raycast(
      origin,
      unit_dir,
      object.px_geom.any(),
      object.px_transform,
      max_dist,
      physx::PxHitFlag::ePOSITION | physx::PxHitFlag::eNORMAL,
      1,
      &hit);
    
    if (num_hits) {
      assert(hit.normal == -unit_dir);
      
      hits_.push_back({
        .hit = hit,
        .object = &object
      });
    }
  }
  
  // Now if we have hits we need to filter them
  for (auto it = hits_.begin(); it != hits_.end(); ++it) {
    bool occluded = false;
    for (auto oit = hits_.begin(); oit != hits_.end(); ++oit) {
      if (oit != it) {
        physx::PxReal dist = physx::PxGeometryQuery::pointDistance(
          it->hit.position, oit->object->px_geom.any(), oit->object->px_transform);

        // the hit point is on another object as well this means that this
        // hit point should have been culled by this other object
        if (dist < 1e-3) {
          occluded = true;
          break;
        }
      }
    }
    
    if (!occluded) {
      const physx::PxVec3& position = it->hit.position;
      
      results.push_back({
        .id = it->object->id,
        .position = Vector3(position.x, position.y, position.z)
      });
    }
  }
  
  return results;
}

}
