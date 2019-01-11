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
  
  T* FindOrInsert(const P *pub, Actor *actor)
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
  
  void RemoveActor(Actor *actor)
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
  std::unordered_map<const P*, std::pair<T*, Actor*>> items_;
  std::unordered_map<Actor*, std::vector<const P*>> by_actor_;
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
  }
  
  virtual ~PxSimulation() {
    scene_->release();
  }
  
  virtual void AddActor(std::shared_ptr<Actor> actor)
  {
    // Build the physx version of the actor
    switch (actor->Type()) {
      case kRigidDynamic:
      {
        RigidDynamic *subject = static_cast<RigidDynamic *>(actor.get());
        const auto& shapes = subject->Shapes();
        for (auto it = shapes.begin(); it != shapes.end(); ++it) {
          std::shared_ptr<const Material> material = it->get()->Material();
          physx::PxMaterial *px_material = factory_.materials.FindOrInsert(material.get(), actor.get());
          px_material = factory_.materials.FindOrInsert(material.get(), actor.get());
        }
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
  physx::PxPhysics *physics_;
  physx::PxScene *scene_;
  PxFactory factory_;
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
    gMaterial = gPhysics->createMaterial(0.5f, 0.5f, 0.6f);

    PxRigidStatic* groundPlane = PxCreatePlane(*gPhysics, PxPlane(0,1,0,0), *gMaterial);
    gScene->addActor(*groundPlane);

    for(PxU32 i=0;i<5;i++)
      createStack(PxTransform(PxVec3(0,0,stackZ-=10.0f)), 10, 2.0f);

    if(!interactive)
      createDynamic(PxTransform(PxVec3(0,40,100)), PxSphereGeometry(10), PxVec3(0,-50,-100));
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
