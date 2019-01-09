
#ifndef HSIM_PHYSICS_HPP
#define HSIM_PHYSICS_HPP

#include "hsim/Geometry.hpp"
#include "hsim/Math.hpp"

#include <memory>

namespace hsim {

class Material {
public:
  virtual ~Material() {}
  virtual Real Restitution() const = 0;
  virtual Real StaticFriction() const = 0;
  virtual Real DynamicFriction() const = 0;
};

class Shape {
public:
  virtual ~Shape() {}
  virtual const Geometry& Geometry() const = 0;
  virtual const Material& Material() const = 0;
};

enum ActorType {
  kRigidStatic,
  kRigidDynamic
};

class Actor {
public:
  virtual ~Actor() {}
  virtual ActorType Type() const = 0;
};

class RigidActor : public Actor {
public:
  virtual ~RigidActor() {}
  virtual const std::vector<Shape>& Shapes() const = 0;
};

class RigidDynamic : public RigidActor {
public:
  virtual ActorType Type() const { return ActorType::kRigidDynamic }
  virtual Real Mass() const = 0;
  virtual const Vector3& CenterOfMass() const = 0;
  virtual const Matrix3& InertiaTensor() const = 0;
};

class Simulation {
public:
  virtual ~Simulation() {}
  virtual void AddActor(std::shared_ptr<Actor> actor) = 0;
  virtual void RemoveActor(std::shared_ptr<Actor> actor) = 0;
};

class PhysicsEngine {
public:
  virtual ~PhysicsEngine() {}
  virtual std::unique_ptr<Simulation> CreateSimulation() = 0;
};

} // namespace hsim

#endif HSIM_PHYSICS_HPP
