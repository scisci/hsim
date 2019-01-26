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

class RayCastQuery {
public:
  typedef std::size_t ObjectID;
  
  struct Result {
    ObjectID object_id;
    Vector3 point;
  };
  
  ObjectID Add(const Geometry& geom, const Transform& transform);
  
  std::vector<Result> Query(const Ray& ray);

};

} // namespace hsim

#endif /* HSIM_COLLISION_HPP */
