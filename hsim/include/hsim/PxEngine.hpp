//
//  NvidiaEngine.hpp
//  hsimall
//
//  Created by z on 1/11/19.
//

#ifndef HSIM_PX_ENGINE_HPP
#define HSIM_PX_ENGINE_HPP

#include "PxPhysicsAPI.h"

#include "hsim/Physics.hpp"
#include "hsim/Scene.hpp"

#define PVD_HOST "127.0.0.1"  //Set this to the IP address of the system running the PhysX Visual Debugger that you want to connect to.


namespace hsim {

template <class P, class T>
class PxLookup {
public:
  PxLookup(std::function<T*(const P*)> factory)
  :factory_(factory)
  {}
  
  virtual ~PxLookup() {
    for (auto it = by_actor_.begin(); it != by_actor_.end(); ++it) {
      RemoveActor(it->first);
    }
  }
  
  T* FindOrInsert(const P *pub, const Actor *actor)
  {
    auto it = items_.find(pub);
    if (it == items_.end()) {
      T *api = factory_(pub);
      items_.insert(std::make_pair(pub, std::make_pair(api, actor)));
      std::vector<const P*> bitems = {pub};
      by_actor_.insert(std::make_pair(actor, bitems));
      return api;
    }
    
    return it->second.first;
  }
  
  void RemoveActor(const Actor *actor)
  {
    auto it = by_actor_.find(actor);
    if (it == by_actor_.end()) {
      return;
    }
    
    for (auto ts = it->second.begin(); ts != it->second.end(); ++ts) {
      auto item_it = items_.find(*ts);
      item_it->second.first->release();
      items_.erase(item_it);
    }
    
    by_actor_.erase(it);
  }
  
private:
  std::unordered_map<const P*, std::pair<T*, const Actor*>> items_;
  std::unordered_map<const Actor*, std::vector<const P*>> by_actor_;
  std::function<T*(const P*)> factory_;
};

class PxFactory {
public:
  PxFactory(physx::PxPhysics *physics)
  : materials([this](const Material *material) {
      return physics_->createMaterial(
        material->StaticFriction(),
        material->KineticFriction(),
        material->Restitution());
    }),
    physics_(physics)
  {}
  
  PxLookup<Material, physx::PxMaterial> materials;
  
private:
  physx::PxPhysics *physics_;
};

class PxScene : public Scene {
public:
/*
  virtual void AddBoxCluster(const BoxCluster& box_cluster)
  {
    
  }
  */
};

class PxSimulation : public Simulation {
public:
  PxSimulation(physx::PxPhysics* physics, const physx::PxSceneDesc& scene_desc)
  : physics_(physics),
    scene_(physics_->createScene(scene_desc)),
    factory_(physics)
  {
    physx::PxPvdSceneClient* pvd_client = scene_->getScenePvdClient();
    if (pvd_client) {
      pvd_client->setScenePvdFlag(physx::PxPvdSceneFlag::eTRANSMIT_CONSTRAINTS, true);
      pvd_client->setScenePvdFlag(physx::PxPvdSceneFlag::eTRANSMIT_CONTACTS, true);
      pvd_client->setScenePvdFlag(physx::PxPvdSceneFlag::eTRANSMIT_SCENEQUERIES, true);
    }
    
    floor_material_ = physics_->createMaterial(0.5f, 0.5f, 0.6f);

    physx::PxRigidStatic* groundPlane = PxCreatePlane(*physics_, physx::PxPlane(0,1,0,0), *floor_material_);
    scene_->addActor(*groundPlane);
    //groundPlane->release();
  }
  
  virtual ~PxSimulation() {
    scene_->release();
  }
  
  physx::PxPhysics* PhysicsRoot()
  {
    return physics_;
  }
  
  physx::PxScene* Scene()
  {
    return scene_;
  }
  
  virtual void AddActor(std::shared_ptr<Actor> actor)
  {
    // Build the physx version of the actor
    switch (actor->Type()) {
      case kRigidDynamic:
      {
        RigidDynamic *subject = static_cast<RigidDynamic *>(actor.get());
        physx::PxTransform transform = ConvertTransform(subject->Transform());
        physx::PxRigidDynamic* body = physics_->createRigidDynamic(transform);
        const auto& shapes = subject->Shapes();
        for (auto it = shapes.begin(); it != shapes.end(); ++it) {
          physx::PxShape *shape = CreateShape(*(it->get()), subject);
          body->attachShape(*shape);
          shape->release();
        }
        physx::PxRigidBodyExt::updateMassAndInertia(*body, 737.0f);
        scene_->addActor(*body);
        //body->release();
      }
      default:
        // Can't create
        return;
    }
  }
  
  virtual void RemoveActor(std::shared_ptr<Actor> actor)
  {
  
  }
  
  virtual void Step(double time_step)
  {
    scene_->simulate(time_step);
    scene_->fetchResults(true);
  }
  
private:
  physx::PxTransform ConvertTransform(const hsim::Transform& transform)
  {
    float values[16];
    const hsim::Transform::MatrixType& matrix = transform.matrix();
    for (int i = 0; i < 16; i++) {
      values[i] = matrix(i);
    }
    return physx::PxTransform(physx::PxMat44(values));
  }
  
  physx::PxShape* CreateShape(const Shape& shape, const Actor* handle)
  {
    physx::PxShape *px_shape = nullptr;
    
    const Geometry& geometry = shape.Geometry();
    std::shared_ptr<const Material> material = shape.Material();
    physx::PxMaterial *px_material = factory_.materials.FindOrInsert(material.get(), handle);
    
    switch (geometry.Type()) {
      case kBox:
      {
        const Box& box = static_cast<const Box&>(geometry);
        px_shape = physics_->createShape(
          physx::PxBoxGeometry(box.Width() * 0.5, box.Height() * 0.5, box.Depth() * 0.5),
          *px_material);
      }
      break;
      case kSphere:
      {
        const Sphere& sphere = static_cast<const Sphere&>(geometry);
        px_shape = physics_->createShape(
          physx::PxSphereGeometry(sphere.Radius()),
          *px_material);
      }
      break;
      default:
        assert(false);
    }
    
    px_shape->setLocalPose(ConvertTransform(shape.Transform()));
    return px_shape;
  }
  
  physx::PxPhysics *physics_;
  physx::PxScene *scene_;
  PxFactory factory_;
  physx::PxMaterial *floor_material_;
};

class PxEngine : public PhysicsEngine {
public:
  PxEngine()
  : foundation_(nullptr),
    physics_(nullptr),
    dispatcher_(nullptr),
    pvd_(nullptr)
  {
    foundation_ = PxCreateFoundation(PX_PHYSICS_VERSION, allocator_, error_callback_);
    pvd_ = physx::PxCreatePvd(*foundation_);
    transport_ = physx::PxDefaultPvdSocketTransportCreate(PVD_HOST, 5425, 10);
    pvd_->connect(*transport_,physx::PxPvdInstrumentationFlag::eALL);
    physics_ = PxCreatePhysics(PX_PHYSICS_VERSION, *foundation_, physx::PxTolerancesScale(),true,pvd_);
    dispatcher_ = physx::PxDefaultCpuDispatcherCreate(2);
    
    

    
    /*
    
  }
*/
  }
  
  virtual ~PxEngine() {
    dispatcher_->release();
    physics_->release();
    pvd_->release();
    transport_->release();
    foundation_->release();
  }
  
  virtual Simulation* InitSimulation()
  {
    physx::PxSceneDesc scene_desc(physics_->getTolerancesScale());
    scene_desc.gravity = physx::PxVec3(0.0f, -9.81f, 0.0f);
    scene_desc.cpuDispatcher = dispatcher_;
    scene_desc.filterShader = physx::PxDefaultSimulationFilterShader;
    
    simulations_.push_back(std::unique_ptr<PxSimulation>(new PxSimulation(physics_, scene_desc)));
    return simulations_.back().get();
  }
  
  virtual void DisposeSimulation(Simulation *simulation)
  {
    for (auto it = simulations_.begin(); it != simulations_.end(); ++it) {
      if (it->get() == simulation) {
        simulations_.erase(it);
        return;
      }
    }
  }

private:
  physx::PxDefaultAllocator allocator_;
  physx::PxDefaultErrorCallback error_callback_;
  physx::PxFoundation *foundation_;
  physx::PxPhysics *physics_;
  physx::PxDefaultCpuDispatcher *dispatcher_;
  physx::PxPvdTransport *transport_;
  physx::PxPvd *pvd_;

  std::vector<std::unique_ptr<PxSimulation>> simulations_;
};

} // namespace hsim

#endif // HSIM_NVIDIA_ENGINE_HPP
