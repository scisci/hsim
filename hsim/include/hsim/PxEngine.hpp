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
      RemoveActor(*(it->first));
    }
  }
  
  T* FindOrInsert(const P *pub, const Actor& actor)
  {
    auto it = items_.find(pub);
    if (it == items_.end()) {
      T *api = factory_(pub);
      items_.insert(std::make_pair(pub, std::make_pair(api, &actor)));
      std::vector<const P*> bitems = {pub};
      by_actor_.insert(std::make_pair(&actor, bitems));
      return api;
    }
    
    return it->second.first;
  }
  
  void RemoveActor(const Actor& actor)
  {
    auto it = by_actor_.find(&actor);
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
  
  static physx::PxGeometryHolder CreateGeometry(const Geometry& geometry)
  {
    physx::PxGeometryHolder holder;
    
    switch (geometry.Type()) {
      case kBox:
      {
        const Box& box = static_cast<const Box&>(geometry);
        holder.storeAny(physx::PxBoxGeometry(box.Width() * 0.5, box.Height() * 0.5, box.Depth() * 0.5));
      }
      break;
      case kSphere:
      {
        const Sphere& sphere = static_cast<const Sphere&>(geometry);
        holder.storeAny(physx::PxSphereGeometry(sphere.Radius()));
      }
      break;
      default:
        assert(false);
    }
    
    return holder;
  }
  
  static physx::PxTransform CreateTransform(const hsim::Transform& transform)
  {
    // TODO: might be able to do this faster by creating a
    // quaternion and a translation
    float values[16];
    const hsim::Transform::MatrixType& matrix = transform.matrix();
    for (int i = 0; i < 16; i++) {
      values[i] = matrix(i);
    }
    return physx::PxTransform(physx::PxMat44(values));
  }
  
  PxLookup<Material, physx::PxMaterial> materials;
private:
  physx::PxPhysics *physics_;
};

enum PxUserDataType {
  kRigidDynamicActorAgent,
  kCharacterController
};

struct PxUserData {
  PxUserDataType type;
  void *inst;
};

class PxActorAgent : public ActorAgent {
public:
  virtual physx::PxActor& Instance() const = 0;
  virtual PxUserData* UserData() = 0;
  virtual void HandleDidSleep() = 0;
};



class PxTransformHandle : public TransformHandle {
public:
  PxTransformHandle(const physx::PxRigidActor& actor)
  : actor_(&actor)
  {}
  
  virtual Vector3 Up() const
  {
    physx::PxVec3 up = actor_->getGlobalPose().rotate(physx::PxVec3(0, 1, 0));
    return Vector3(up.x, up.y, up.z);
  }
  
  virtual Real Tilt() const {
    const physx::PxVec3 y_axis(0, 1, 0);
    const physx::PxVec3 up = actor_->getGlobalPose().rotate(y_axis);
    // It is necessary to normalize here since float precision errors,
    // otherwise it falls outside the acos domain.
    const physx::PxReal cos_theta = up.dot(y_axis) / up.magnitude();
    return acos(cos_theta);
  }
  
private:

  const physx::PxRigidActor *actor_;
};

class PxRigidDynamicAgent : public PxActorAgent {
public:
  PxRigidDynamicAgent(const Actor& model, physx::PxRigidDynamic *inst)
  : model_(&model),
    inst_(inst),
    transform_(*inst),
    user_data_({.type = kRigidDynamicActorAgent, .inst = this})
  {}
  
  ~PxRigidDynamicAgent()
  {
    inst_->release();
  }
  
  virtual const Actor& Model() const
  {
    return *model_;
  }
  
  virtual const TransformHandle& Transform() const
  {
    return transform_;
  }
  
  virtual physx::PxActor& Instance() const
  {
    return *inst_;
  }
  
  virtual PxUserData* UserData()
  {
    return &user_data_;
  }
  
  virtual void AddImpulseAtLocalPos(const Vector3& force, const Vector3& pos)
  {
    physx::PxRigidBodyExt::addForceAtLocalPos(
      *inst_,
      physx::PxVec3(force.x(), force.y(), force.z()),
      physx::PxVec3(pos.x(), pos.y(), pos.z()),
      physx::PxForceMode::eIMPULSE);
  }
  
  
  virtual boost::signals2::connection ConnectDidSleep(
    const DidSleepSignal::slot_type& slot)
  {
    if (sleep_signal_.empty()) {
      inst_->setActorFlags(
          inst_->getActorFlags() | physx::PxActorFlag::eSEND_SLEEP_NOTIFIES);
    }
    
    return sleep_signal_.connect(slot);
  }
  
  virtual boost::signals2::connection ConnectDidAdvance(
    const DidAdvanceSignal::slot_type& slot)
  {
    return advance_signal_.connect(slot);
  }
  
  virtual void HandleDidSleep()
  {
    sleep_signal_();
  }
  
  virtual void HandleDidAdvance()
  {
    advance_signal_();
  }
  
private:
  const Actor* model_;
  physx::PxRigidDynamic *inst_;
  PxTransformHandle transform_;
  
  DidSleepSignal sleep_signal_;
  DidAdvanceSignal advance_signal_;
  
  PxUserData user_data_;
};


class PxCharacterController : public Character {
public:
  PxCharacterController(physx::PxController *inst)
  : inst_(inst),
    user_data_({.type = kCharacterController, .inst = this})
  {
    filters_.mFilterFlags = physx::PxQueryFlag::eSTATIC | physx::PxQueryFlag::eDYNAMIC;
  }
  
  ~PxCharacterController()
  {
    inst_->release();
  }
  
  physx::PxController* Instance()
  {
    return inst_;
  }
  
  PxUserData* UserData()
  {
    return &user_data_;
  }
  
  virtual hsim::Vector3 Position() const
  {
    auto pos = inst_->getPosition();
    return hsim::Vector3(pos.x, pos.y, pos.z);
  }
  
  virtual void SetPosition(const Vector3& position)
  {
    inst_->setPosition(physx::PxExtendedVec3(position.x(), position.y(), position.z()));
  }
  
  virtual void Move(const Vector3& offset)
  {
    const physx::PxReal min_dist = 0.001; // Min error for reaching end point
    const physx::PxReal elapsed_time = 1.0f / 60.0f;
    physx::PxControllerCollisionFlags flags = inst_->move(
      physx::PxVec3(offset.x(), offset.y(), offset.z()),
      min_dist,
      elapsed_time,
      filters_,
      nullptr);
    
    if (flags) {
      std::cout << "colliding " << std::endl;
    }
  }
  
private:
  physx::PxController *inst_;
  PxUserData user_data_;
  physx::PxControllerFilters filters_;
};


template <typename T>
struct UniquePtrComparator {
  bool operator() (const std::unique_ptr<T>& lhs, const std::unique_ptr<T>& rhs) const
  {
    return lhs < rhs;
  }
  
  bool operator() (const std::unique_ptr<T>& lhs, const T* rhs) const
  {
    return lhs.get() < rhs;
  }
  
  bool operator() (const T* lhs, const std::unique_ptr<T>& rhs) const
  {
    return lhs < rhs.get();
  }
};

enum PxSimulationEventFlag {
  kSleep = 1
};

class PxSimulation : public Simulation, public physx::PxSimulationEventCallback {
public:
  PxSimulation(physx::PxPhysics* physics, const physx::PxSceneDesc& scene_desc)
  : physics_(physics),
    scene_(physics_->createScene(scene_desc)),
    factory_(physics),
    ctrl_manager_(PxCreateControllerManager(*scene_))
  {
    scene_->setSimulationEventCallback(this);
    
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
    ctrl_manager_->release();
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
  
  virtual Character* AddCharacter(hsim::Real radius, hsim::Real height)
  {
    physx::PxCapsuleControllerDesc desc;
    desc.radius = radius;
    desc.height = height;
    desc.climbingMode = physx::PxCapsuleClimbingMode::eEASY;
    desc.material = floor_material_;
    physx::PxController* ctrl = ctrl_manager_->createController(desc);
    
    std::unique_ptr<PxCharacterController> character(new PxCharacterController(ctrl));
    
    Character *result = character.get();
    ctrl->getActor()->userData = character->UserData();
    
    auto sorted_pos = std::upper_bound(
      characters_.begin(),
      characters_.end(),
      character.get(),
      UniquePtrComparator<PxCharacterController>());
    
    characters_.insert(sorted_pos, std::move(character));
    return result;
  }
  
  virtual void RemoveCharacter(const Character& character)
  {
    assert(0);
  }
  
  virtual ActorAgent* AddActor(const Actor& actor)
  {
    std::unique_ptr<PxActorAgent> agent;
    
    // Build the physx version of the actor
    switch (actor.Type()) {
      case kRigidDynamic:
      {
        const RigidDynamic& subject = static_cast<const RigidDynamic&>(actor);
        physx::PxTransform transform = ConvertTransform(subject.Transform());
        physx::PxRigidDynamic* body = physics_->createRigidDynamic(transform);
        
        // Store relationship
        physx::PxReal thresh = body->getSleepThreshold();
        physx::PxReal wake_counter = body->getWakeCounter();
        body->setSleepThreshold(0.0001);
        body->setWakeCounter(0.4);
        physx::PxReal damp = body->getAngularDamping();
        body->setAngularDamping(0.005);
        

        const auto& shapes = subject.Shapes();
        for (auto it = shapes.begin(); it != shapes.end(); ++it) {
          physx::PxShape *shape = CreateShape(*(it->get()), subject);
          body->attachShape(*shape);
          shape->release();
        }
        physx::PxRigidBodyExt::updateMassAndInertia(*body, 737.0f);

        agent.reset(new PxRigidDynamicAgent(actor, body));
      }
      break;
      default:
        // Can't create
        return nullptr;
    }
    
    assert(agent != nullptr);
    
    // Store pointer because we are about to move and unique_ptr won't be
    // good anymore
    
    agent->Instance().userData = agent->UserData();
    scene_->addActor(agent->Instance());
    
    ActorAgent *agent_ptr = agent.get();
    
    auto sorted_pos = std::upper_bound(
      actors_.begin(),
      actors_.end(),
      agent.get(),
      UniquePtrComparator<PxActorAgent>());
    
    actors_.insert(sorted_pos, std::move(agent));
    return agent_ptr;
  }
  
  virtual void RemoveActor(const ActorAgent& agent)
  {
    const PxActorAgent& px_agent = static_cast<const PxActorAgent&>(agent);
    
    auto it = std::lower_bound(
      actors_.begin(),
      actors_.end(),
      &px_agent,
      UniquePtrComparator<PxActorAgent>());
    
    if (it == actors_.end() || it->get() != &agent) {
      return;
    }
    
    scene_->removeActor(it->get()->Instance());
    factory_.materials.RemoveActor(it->get()->Model());
    actors_.erase(it);
    
    
  }
  
  virtual void Step(double time_step)
  {
    event_flags_ = 0;
    scene_->simulate(time_step);
    scene_->fetchResults(true);
    if (event_flags_) {
      DispatchEvents();
    }
  }
  
  virtual void onConstraintBreak(physx::PxConstraintInfo *constraints, physx::PxU32 count)
  {
  
  }
  
  virtual void onWake(physx::PxActor **actors, physx::PxU32 count)
  {
  
  }
  
  virtual void onSleep(physx::PxActor **actors, physx::PxU32 count)
  {
    event_flags_ |= PxSimulationEventFlag::kSleep;
    sleep_buffer_.resize(count);
    std::copy(actors, actors + count, sleep_buffer_.begin());
  }
  
  virtual void onContact(
    const physx::PxContactPairHeader &pairHeader,
    const physx::PxContactPair *pairs,
    physx::PxU32 nbPairs)
  {
    std::cout << "contact" << std::endl;
  }
  
  virtual void onTrigger(physx::PxTriggerPair *pairs, physx::PxU32 count)
  {
  
  }
  
  virtual void onAdvance(
    const physx::PxRigidBody *const *bodyBuffer,
    const physx::PxTransform *poseBuffer,
    const physx::PxU32 count)
  {
    
  }
  
private:
  void DispatchEvents()
  {
    if (event_flags_ & PxSimulationEventFlag::kSleep) {
      for (auto it = sleep_buffer_.begin(); it != sleep_buffer_.end(); ++it) {
        physx::PxActor *actor = *it;
        assert(actor->userData != nullptr);
        PxUserData *user_data = static_cast<PxUserData *>(actor->userData);
        if (user_data->type == kRigidDynamicActorAgent) {
          PxActorAgent *agent = static_cast<PxActorAgent *>(user_data->inst);
          agent->HandleDidSleep();
        }
      }
      sleep_buffer_.clear();
    }
  }
  physx::PxTransform ConvertTransform(const hsim::Transform& transform)
  {
    return PxFactory::CreateTransform(transform);
  }
  
  physx::PxShape* CreateShape(const Shape& shape, const Actor& handle)
  {
    
    //const Geometry& geometry = shape.Geometry();
    std::shared_ptr<const Material> material = shape.Material();
    physx::PxMaterial *px_material = factory_.materials.FindOrInsert(material.get(), handle);
    
    physx::PxGeometryHolder holder = PxFactory::CreateGeometry(shape.Geometry());
    physx::PxShape *px_shape = physics_->createShape(holder.any(), *px_material);
    /*
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
    */
    px_shape->setLocalPose(ConvertTransform(shape.Transform()));
    return px_shape;
  }
  
  physx::PxPhysics *physics_;
  physx::PxScene *scene_;
  PxFactory factory_;
  physx::PxMaterial *floor_material_;
  physx::PxControllerManager *ctrl_manager_;
  
  int event_flags_;
  std::vector<physx::PxActor *> sleep_buffer_;
  
  std::vector<std::unique_ptr<PxActorAgent>> actors_;
  std::vector<std::unique_ptr<PxCharacterController>> characters_;
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
    dispatcher_ = physx::PxDefaultCpuDispatcherCreate(4);
    
    

    
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
  
  virtual std::unique_ptr<Simulation> CreateSimulation()
  {
    physx::PxSceneDesc scene_desc(physics_->getTolerancesScale());
    scene_desc.gravity = physx::PxVec3(0.0f, -9.81f, 0.0f);
    scene_desc.cpuDispatcher = dispatcher_;
    scene_desc.filterShader = physx::PxDefaultSimulationFilterShader;
    
    return std::unique_ptr<PxSimulation>(new PxSimulation(physics_, scene_desc));
  }
  
  
  

private:
  physx::PxDefaultAllocator allocator_;
  physx::PxDefaultErrorCallback error_callback_;
  physx::PxFoundation *foundation_;
  physx::PxPhysics *physics_;
  physx::PxDefaultCpuDispatcher *dispatcher_;
  physx::PxPvdTransport *transport_;
  physx::PxPvd *pvd_;
};

} // namespace hsim

#endif // HSIM_NVIDIA_ENGINE_HPP
