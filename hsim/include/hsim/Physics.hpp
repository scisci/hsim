
#ifndef HSIM_PHYSICS_HPP
#define HSIM_PHYSICS_HPP

#include "hsim/Geometry.hpp"
#include "hsim/Math.hpp"
#include "hsim/Colors.hpp"

#include <boost/signals2.hpp>
#include <memory>

namespace hsim {

class Actor;

class RenderDecorator {
public:
  virtual ~RenderDecorator() {}
  virtual const std::vector<Color>* ActorColorMap(const Actor& actor) const = 0;
};

struct MaterialCoefficients {
  Real restitution;
  Real static_friction;
  Real kinetic_friction;
};

class Material {
public:
  virtual ~Material() {}
  virtual Real Restitution() const = 0;
  virtual Real StaticFriction() const = 0;
  virtual Real KineticFriction() const = 0;
};

class Shape {
public:
  virtual ~Shape() {}
  virtual const Geometry& Geometry() const = 0;
  virtual std::shared_ptr<const Material> Material() const = 0;
  virtual const Transform& Transform() const = 0;
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

class TransformHandle {
public:
  virtual ~TransformHandle() {}
  virtual Vector3 Up() const = 0;
  virtual Real Tilt() const = 0;
  
};


class Character {
public:
  virtual ~Character() {}
  virtual Vector3 Position() const = 0;
  virtual void SetPosition(const Vector3& position) = 0;
  virtual void Move(const Vector3& offset) = 0;
};

class ActorAgent {
public:
  typedef boost::signals2::signal<void ()> DidSleepSignal;
  typedef boost::signals2::signal<void ()> DidAdvanceSignal;
  
  virtual ~ActorAgent() {}
  virtual const Actor& Model() const = 0;
  virtual const TransformHandle& Transform() const = 0;
  virtual void AddImpulseAtLocalPos(const Vector3& force, const Vector3& pos) = 0;
  
  virtual boost::signals2::connection ConnectDidSleep(
    const DidSleepSignal::slot_type& slot) = 0;
  virtual boost::signals2::connection ConnectDidAdvance(
    const DidAdvanceSignal::slot_type& slot) = 0;
};

class RigidActor : public Actor {
public:
  virtual ~RigidActor() {}
  virtual const std::vector<std::unique_ptr<Shape>>& Shapes() const = 0;
  
  static AlignedBox BoundingBox(
    const RigidActor& actor,
    const Transform& transform)
  {
    AlignedBox box;
    for (auto& shape : actor.Shapes()) {
      auto t = transform * shape->Transform();
      AlignedBox geom_box = shape->Geometry().BoundingBox();
      box.extend(t * geom_box.corner(AlignedBox::CornerType::TopLeftFloor));
      box.extend(t * geom_box.corner(AlignedBox::CornerType::TopRightFloor));
      box.extend(t * geom_box.corner(AlignedBox::CornerType::BottomRightFloor));
      box.extend(t * geom_box.corner(AlignedBox::CornerType::BottomLeftFloor));
      box.extend(t * geom_box.corner(AlignedBox::CornerType::TopLeftCeil));
      box.extend(t * geom_box.corner(AlignedBox::CornerType::TopRightCeil));
      box.extend(t * geom_box.corner(AlignedBox::CornerType::BottomRightCeil));
      box.extend(t * geom_box.corner(AlignedBox::CornerType::BottomLeftCeil));
    }
    return box;
  }
};



class RigidBody : public RigidActor {
public:
  virtual Real Mass() const = 0;
  virtual const Vector3& CenterOfMass() const = 0;
  virtual const Matrix3& InertiaTensor() const = 0;
  virtual const Transform& Transform() const = 0;
  //virtual Vector6 SpatialVelocity() const = 0;
  
  /*
  double BodyNode::computeKineticEnergy() const
{
  const Eigen::Vector6d& V = getSpatialVelocity();
  const Eigen::Matrix6d& G = mAspectProperties.mInertia.getSpatialTensor();

  return 0.5 * V.dot(G * V);
}
  */
  
  // vs
  
  /*
  const PxVec3 t = core.getInverseInertia();
      const PxVec3 inertia(t.x > 0.0f ? 1.0f/t.x : 1.0f, t.y > 0.0f ? 1.0f/t.y : 1.0f, t.z > 0.0f ? 1.0f/t.z : 1.0f);

      PxVec3 sleepLinVelAcc =motionVelocity.linear;
      PxVec3 sleepAngVelAcc = body2World.q.rotateInv(motionVelocity.angular);

      bcSleepLinVelAcc += sleepLinVelAcc;
      bcSleepAngVelAcc += sleepAngVelAcc;

      PxReal invMass = core.getInverseMass();
      if(invMass == 0.0f)
        invMass = 1.0f;

      const PxReal angular = bcSleepAngVelAcc.multiply(bcSleepAngVelAcc).dot(inertia) * invMass;
      const PxReal linear = bcSleepLinVelAcc.magnitudeSquared();
      PxReal normalizedEnergy = 0.5f * (angular + linear);
      */
};

class RigidDynamic : public RigidBody {
public:
  virtual ActorType Type() const { return ActorType::kRigidDynamic; }
};

class RigidBodyImpl : public RigidDynamic {
public:
  RigidBodyImpl(
      Real mass,
      const Vector3& center_of_mass,
      const Matrix3& inertia_tensor,
      const hsim::Transform& transform,
      std::vector<std::unique_ptr<Shape>>&& shapes)
  : mass_(mass),
    center_of_mass_(center_of_mass),
    inertia_tensor_(inertia_tensor),
    transform_(transform),
    shapes_(std::move(shapes))
  {}

  virtual Real Mass() const { return mass_; }
  virtual const Vector3& CenterOfMass() const { return center_of_mass_; }
  virtual const Matrix3& InertiaTensor() const { return inertia_tensor_; }
  virtual const hsim::Transform& Transform() const { return transform_; }
  virtual const std::vector<std::unique_ptr<Shape>>& Shapes() const { return shapes_; }

private:
  Real mass_;
  Vector3 center_of_mass_;
  Matrix3 inertia_tensor_;
  hsim::Transform transform_;
  std::vector<std::unique_ptr<Shape>> shapes_;
};

class MaterialImpl : public Material {
public:
  MaterialImpl(const MaterialCoefficients& coefficients)
  : coefficients_(coefficients)
  {}

  virtual Real Restitution() const { return coefficients_.restitution; }
  virtual Real StaticFriction() const { return coefficients_.static_friction; }
  virtual Real KineticFriction() const { return coefficients_.kinetic_friction; }

private:
  MaterialCoefficients coefficients_;
};

class ShapeImpl : public Shape {
public:
  ShapeImpl(
      std::unique_ptr<hsim::Geometry> geometry,
      const hsim::Transform& transform,
      std::shared_ptr<hsim::MaterialImpl> material)
  : geometry_(std::move(geometry)),
    material_(material),
    transform_(transform)
  {}
  
  virtual const class Geometry& Geometry() const { return *geometry_.get(); }
  virtual std::shared_ptr<const class Material> Material() const { return material_; }
  virtual const hsim::Transform& Transform() const { return transform_; }
  
private:
  std::unique_ptr<hsim::Geometry> geometry_;
  std::shared_ptr<hsim::Material> material_;
  hsim::Transform transform_;
};

class RigidBodyBuilder {
public:
  RigidBodyBuilder()
  : transform_(Transform::Identity()),
    material_(new MaterialImpl({
      .restitution = 1.0,
      .static_friction = 0.25,
      .kinetic_friction = 0.25}))
  {}
  
  static std::unique_ptr<RigidBody> Build(std::unique_ptr<hsim::Geometry> geometry, Real density, const Transform& transform)
  {
    RigidBodyBuilder builder;
    builder.AddShape(std::move(geometry), density, transform);
    return builder.Build();
  }
  
  void AddShape(std::unique_ptr<hsim::Geometry> geometry, Real density, const Transform& transform)
  {
    Real volume = MassProperties::ComputeVolume(*geometry.get());
    
    shapes_.push_back({
      .geometry = std::move(geometry),
      .density = density,
      .transform = transform,
      .mass = density * volume,
      .center_of_mass = transform.translation()
    });
  }
  
  void SetMaterial(const MaterialCoefficients& coefficients)
  {
    material_ = std::make_shared<MaterialImpl>(coefficients);
  }
  
  void SetTransform(const Transform& transform)
  {
    transform_ = transform;
  }
  
  std::unique_ptr<RigidBody> Build()
  {
    Vector3 total_com = Vector3::Zero();
    Real total_mass = 0.0;
    Matrix3 total_inertia = Matrix3::Zero();
    std::vector<std::unique_ptr<Shape>> shapes;
    
    for (auto& shape : shapes_) {
      total_com += shape.center_of_mass * shape.mass;
      total_mass += shape.mass;
    }
    
    total_com /= total_mass;
    
    for (auto& shape : shapes_) {
      Matrix3 inertia = MassProperties::ComputeInertia(*shape.geometry.get(), shape.mass);
      Vector3 offset = shape.center_of_mass - total_com;
      total_inertia += ParallelAxisTheorem(inertia, offset, shape.mass);
    }
    
    // Shapes is now invalid since we moved the geometry, so clear
    // it. The builder can be reused, but will be empty.
    for (auto& shape : shapes_) {
      shapes.push_back(std::unique_ptr<Shape>(
        new ShapeImpl(
          std::move(shape.geometry),
          shape.transform,
          material_)));
    }
    
    shapes_.clear();
    
    return std::unique_ptr<RigidBody>(
      new RigidBodyImpl(
        total_mass,
        total_com,
        total_inertia,
        transform_,
        std::move(shapes)));
  }
  
private:

  struct ShapeProps {
    std::unique_ptr<hsim::Geometry> geometry;
    Real density;
    Transform transform;
    
    // Used for calculation only
    Real mass;
    Vector3 center_of_mass;
  };
  
  Matrix3 ParallelAxisTheorem(const Matrix3& original, const Vector3& offset, Real mass)
  {
    Matrix3 result(original);
    for (std::size_t i = 0; i < 3; ++i) {
      for (std::size_t j = 0; j < 3; ++j) {
        result(i, j) += mass * ((i == j ? 1 : 0) * offset.dot(offset) - offset(i) * offset(j));
      }
    }
    return result;
  }
  
  Transform transform_;
  std::shared_ptr<MaterialImpl> material_;
  std::vector<ShapeProps> shapes_;
};



class Simulation {
public:
  virtual ~Simulation() {}
  virtual ActorAgent* AddActor(const Actor& actor) = 0;
  virtual void RemoveActor(const ActorAgent& actor_agent) = 0;
  virtual Character* AddCharacter(Real radius, Real height) = 0;
  virtual void RemoveCharacter(const Character& character) = 0;
  virtual void Step(double time_step) = 0;
};

class PhysicsEngine {
public:
  virtual ~PhysicsEngine() {}
  virtual std::unique_ptr<Simulation> CreateSimulation() = 0;
};

} // namespace hsim

#endif // HSIM_PHYSICS_HPP
