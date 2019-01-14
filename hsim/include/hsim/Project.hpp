
#ifndef HSIM_PROJECT_HPP
#define HSIM_PROJECT_HPP

#include "hsim/Physics.hpp"

#include "htree/Golden.hpp"
#include "htree/RandomBasicGenerator.hpp"
#include "htree/RegionIterator.hpp"
#include "htree/EdgePathAttributer.hpp"

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
    state_(0)
  {
  
  }
  
  void Next()
  {
    // Remove everything from the scene
    for (auto it = agents_.begin(); it != agents_.end(); ++it) {
      simulation_->RemoveActor(**it);
    }
    
    for (auto& conn : conns_) {
      conn.disconnect();
    }
    conns_.clear();
    agents_.clear();
    actors_.clear();
    state_ = 0;
    
    hsim::Project project;
    std::unique_ptr<htree::Tree> tree = project.GenerateTree();
    htree::StringNodeAttributes attributes = project.Attribute(*tree.get());
    std::unique_ptr<Actor> actor = project.CreateActor(*tree.get(), attributes);
    ActorAgent *agent = simulation_->AddActor(*(actor.get()));
    
    // Add an initial nudge

    //agent->AddImpulseAtLocalPos(Vector3(20, 0, 0), Vector3(0, 2.0, 0));
    
    // Listen for the agent
    conns_.push_back(
      agent->ConnectDidSleep(
        std::bind(&Iteration::HandleSleepCallback, this, agent)));
    
    actors_.push_back(std::move(actor));
    agents_.push_back(agent);
  }
  
  
  void HandleSleepCallback(hsim::ActorAgent *agent)
  {
    hsim::Real tilt = agent->Transform().Tilt();
    
    // If tilt is greater than 10 degreees then it fails
    if (tilt > 10.0 * M_PI / 180.0) {
      std::cout << "Fell over" << std::endl;
      state_ = 4;
    } else {
      std::cout << "Upright" << std::endl;
      if (++state_ == 1) {
        agent->AddImpulseAtLocalPos(Vector3(50, 0, 0), Vector3(0, 2.0, 0));
      } else if (state_ == 2) {
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
  
private:
  std::unique_ptr<Simulation> simulation_;
  std::vector<std::unique_ptr<Actor>> actors_;
  std::vector<ActorAgent *> agents_;
  int state_;
  std::vector<boost::signals2::connection> conns_;
};


} // namespace hsim

#endif // HSIM_PROJECT_HPP
