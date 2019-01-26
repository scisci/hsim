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

class RaycastQuery {
public:
  RaycastQuery();
  
  typedef std::size_t ObjectID;
  
  struct Result {
    ObjectID id;
    Vector3 position;
  };
  
  ObjectID Add(const Geometry& geom, const Transform& transform);
  
  std::vector<Result> Query(const Ray& ray);

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
  std::vector<Hit> hits_;
};

} // namespace hsim

#endif /* HSIM_COLLISION_HPP */
