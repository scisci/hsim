
#ifndef HSIM_PROJECT_HPP
#define HSIM_PROJECT_HPP

#include "hsim/Physics.hpp"

#include "htree/Golden.hpp"
#include "htree/RandomBasicGenerator.hpp"
#include "htree/RegionIterator.hpp"
#include "htree/EdgePathAttributer.hpp"

#include "hsim/Tess.hpp";


#include <memory>
#include <iostream>

namespace hsim {

struct IterationParams {
  std::unique_ptr<htree::Tree> tree;
  htree::StringNodeAttributes attributes;
};

struct IterationResult {
  bool upright;
};



class Project {
public:
  Project()
  :rng(rd())
  {}
  
  virtual ~Project() {}
  
  // Should generate a new iteration of the tree specified
  std::unique_ptr<htree::Tree> GenerateTree();
  htree::StringNodeAttributes Attribute(const htree::Tree& tree);
  std::unique_ptr<Actor> CreateActor(
    const htree::Tree& tree,
    const htree::StringNodeAttributes& attributes);
  
  
private:
  std::random_device rd;
  std::mt19937_64 rng;
};


enum IterationStatus {
  kIncomplete,
  kComplete,
  kFailed
};

class Iteration {
public:
  Iteration(PhysicsEngine& engine)
  : simulation_(engine.CreateSimulation()),
    state_(0),
    agent_(nullptr)
  {
  
  }
  
  void Retry()
  {
    Clear();
    Load();
  }
  
  void Clear()
  {
    if (agent_ == nullptr) {
      return;
    }
    
    simulation_->RemoveActor(*agent_);

    for (auto& conn : conns_) {
      conn.disconnect();
    }
    conns_.clear();
    agent_ = nullptr;
    state_ = 0;
  }
  
  void Next()
  {
    Clear();
    
    hsim::Project project;
    std::unique_ptr<htree::Tree> tree = project.GenerateTree();
    htree::StringNodeAttributes attributes = project.Attribute(*tree.get());
    actor_ = project.CreateActor(*tree.get(), attributes);
    
    Load();
  }
  
  void Load()
  {
    agent_ = simulation_->AddActor(*(actor_.get()));

    conns_.push_back(
      agent_->ConnectDidSleep(
        std::bind(&Iteration::HandleSleepCallback, this, agent_)));
  }
  
  
  void HandleSleepCallback(hsim::ActorAgent *agent)
  {
    hsim::Real tilt = agent->Transform().Tilt();
    std::cout << "Tilt " << (tilt * 180.0 / M_PI) << std::endl;
    // If tilt is greater than 10 degreees then it fails
    if (tilt > 10.0 * M_PI / 180.0) {
      std::cout << "Fell over" << std::endl;
      state_ = 4;
    } else {
      std::cout << "Upright" << std::endl;
      if (++state_ == 1) {
        std::cout << "Push right" << std::endl;
        agent->AddImpulseAtLocalPos(Vector3(50, 0, 0), Vector3(0, 2.0, 0));
      } else if (state_ == 2) {
        std::cout << "Push back" << std::endl;
        agent->AddImpulseAtLocalPos(Vector3(0, 0, 50), Vector3(0, 2.0, 0));
      }
    }
    
  }
  
  IterationStatus Step()
  {
    simulation_->Step(1.0 / 60.0);
    if (state_ == 4) {
      return kFailed;
    }
    
    return state_ == 3 ? kComplete : kIncomplete;
  }
  
  void Write()
  {
    // Take our actor and tesselate and write it
    if (actor_ == nullptr) {
      return;
    }
    
    if (actor_->Type() != kRigidDynamic && actor_->Type() != kRigidStatic) {
      return;
    }
    
    const RigidActor& rigid_actor = static_cast<const RigidActor&>(*actor_.get());
    
    TesselationBuilder builder;
    
    for (const auto& shape : rigid_actor.Shapes()) {
      const Geometry& geometry = shape->Geometry();
      switch (geometry.Type()) {
        case kBox:
          builder.Add(static_cast<const Box&>(geometry), shape->Transform());
          break;
        default:
          // Only boxes work for now
          assert(0);
      }
    }
    
    StlwFormatWriter writer;
    
    // 2m -> 12cm
    double scale = 12.0 / 200.0 * 1000.0; // Meters -> mm
    AffineTransform transform = AffineTransform::Identity();
    // Since we want z-axis up
    Eigen::AngleAxis<Real> axis_swap(0.5 * M_PI, Vector3::UnitX());
    transform.rotate(axis_swap);
    transform.scale(scale);
    writer.SetTransform(transform);
    writer.Write(builder.Tesselation(), std::cout);
  }
  
private:
  std::unique_ptr<Simulation> simulation_;
  std::unique_ptr<Actor> actor_;
  ActorAgent *agent_;
  int state_;
  std::vector<boost::signals2::connection> conns_;
};


} // namespace hsim

#endif // HSIM_PROJECT_HPP
