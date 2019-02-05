
#ifndef HSIM_PROJECT_HPP
#define HSIM_PROJECT_HPP

#include "hsim/Physics.hpp"

#include "htree/Golden.hpp"
#include "htree/RandomBasicGenerator.hpp"
#include "htree/RegionIterator.hpp"
#include "htree/EdgePathAttributer.hpp"


#include "hsim/Tess.hpp"


#include "hsim/JumpSolver.hpp"
#include "hsim/Plans.hpp"

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


struct Curve {
  std::size_t start_index;
  std::size_t end_index;
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
    agent_(nullptr),
    state_(0),
    sim_time_(0.0),
    debug_agent_(nullptr)
  {
    character_ = simulation_->AddCharacter(0.1, 0.5);
    character_->SetPosition(Vector3(-10.0, 0.0, 0.0));
  }
  
  std::vector<Curve> curves;
  std::vector<hsim::Vector3> curve_verts;
  
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
    
    if (debug_agent_ != nullptr) {
      simulation_->RemoveActor(*debug_agent_);
    }

    for (auto& conn : conns_) {
      conn.disconnect();
    }
    conns_.clear();
    agent_ = nullptr;
    debug_agent_ = nullptr;
    state_ = 0;
    sim_time_ = 0;
  }
  
  void Next()
  {
    Clear();
    
    hsim::Project project;
    std::unique_ptr<htree::Tree> tree = project.GenerateTree();
    htree::StringNodeAttributes attributes = project.Attribute(*tree.get());
    actor_ = project.CreateActor(*tree.get(), attributes);
    
    Load();
    
    Intersect();
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
      RigidDynamic *rigid_actor = static_cast<RigidDynamic *>(actor_.get());
      
      std::cout << "Upright" << std::endl;
      if (++state_ == 1) {
        hsim::Real mass = rigid_actor->Mass();
        std::cout << "Push right against " << mass << std::endl;
        agent->AddImpulseAtLocalPos(Vector3(mass * 0.2, 0, 0), Vector3(0, 2.0, 0));
      } else if (state_ == 2) {
        hsim::Real mass = rigid_actor->Mass();
        std::cout << "Push back against " << rigid_actor->Mass() << std::endl;
        agent->AddImpulseAtLocalPos(Vector3(0, 0, mass * 0.2), Vector3(0, 2.0, 0));
      } else {
        // Done
      }
    }
    
  }
  
  void AddMouseTest(const Vector3& pos)
  {
    RigidBodyBuilder builder;
    Transform transform = Transform::Identity();
    transform.translation() = pos;
    builder.SetTransform(transform);
    builder.AddShape(std::unique_ptr<Box>(new Box(1.0, 1.0, 1.0)), 1000.0, Transform::Identity());
    
    auto result = builder.Build();
    
    simulation_->AddActor(*result.get());
  }
  
  void Intersect()
  {
    if (actor_->Type() != kRigidDynamic && actor_->Type() != kRigidStatic) {
      return;
    }
    
    const RigidBody& rigid_body = static_cast<const RigidBody&>(*actor_.get());

    const Real cat_radius = 0.125;
    jump_solver_.Solve(rigid_body);
    
    auto samples = jump_solver_.Vertices();
    auto paths = jump_solver_.Edges();

    curves.clear();
    curve_verts.clear();

    RigidBodyBuilder builder;
    
    for (int i = 0; i < samples.size(); i++) {
      Transform transform = Transform::Identity();
      transform.translation() = samples[i] + Vector3(0, cat_radius, 0);
      builder.AddShape(std::unique_ptr<Sphere>(new Sphere(cat_radius)), 1000.0, transform);
    }
    
    for (int i = 0; i < paths.size(); ++i) {
      hsim::Real length = paths[i]->Length();
      Curve curve;
      curve.start_index = curve_verts.size();
      for (int p = 0; p <= 20; ++p) {
        curve_verts.push_back(paths[i]->Compute(p * length/20.0) + Vector3(0, 0.25, 0));
      }
      curve.end_index = curve_verts.size();
      curves.push_back(curve);
    }
    
    auto actor = builder.Build();
    debug_agent_ = simulation_->AddActor(*actor.get());
    
    PlanBuilder pbuild;
    pbuild.Build(rigid_body);
  }
  
  IterationStatus Step()
  {
    float dt = 1.0f / 60.0f;
    sim_time_ += dt;
    //simulation_->Step(dt);
    
    //character_->Move(Vector3(0.005, -9.8f / 60.0f, 0));
    
    switch (state_) {
      case 4:
        return kFailed;
      case 3:
        return kComplete;
      default:
        if (sim_time_ >= 5.0) {
          return kComplete;
        }
        return kIncomplete;
    }
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
    
    Vector3 units[] = {
      Vector3::UnitX(),
      Vector3::UnitY(),
      Vector3::UnitZ()
    };
    
    for (const auto& shape : rigid_actor.Shapes()) {
      const Geometry& geometry = shape->Geometry();
      switch (geometry.Type()) {
        case kBox:
        {
          const Box& box = static_cast<const Box&>(geometry);
          //builder.Add(box, shape->Transform());
          
          Real inset = 0.0025; // half cm
          Real depth = 0.0025;
          
          Vector3 orig_sizes(box.Width(), box.Height(), box.Depth());
 
          for (int i = 0; i < 3; ++i) {
            size_t width_index, height_index;
            switch (i) {
              case 0:
                width_index = 2;
                height_index = 1;
                break;
              case 1:
                width_index = 0;
                height_index = 2;
                break;
              case 2:
                width_index = 0;
                height_index = 1;
                break;
            }
            
            
            Vector3 width_vec = units[width_index];
            Vector3 height_vec = units[height_index];
            Vector3 depth_vec = units[i];
            Real width = orig_sizes(width_index);
            Real height = orig_sizes(height_index);
            Real d = orig_sizes(i);
            
            std::vector<Vector3> points(8);
            points[0] = -width/2 * width_vec - height/2 * height_vec;
            points[1] = width/2 * width_vec - height/2 * height_vec;
            points[2] = width/2 * width_vec + height/2 * height_vec;
            points[3] = -width/2 * width_vec + height/2 * height_vec;
            points[4] = -(width/2 - inset) * width_vec - (height/2 - inset) * height_vec;
            points[5] = (width/2 - inset) * width_vec - (height/2 - inset) * height_vec;
            points[6] = (width/2 - inset) * width_vec + (height/2 - inset) * height_vec;
            points[7] = -(width/2 - inset) * width_vec + (height/2 - inset) * height_vec;
            
            std::vector<std::size_t> indices = {
              0, 1, 4, 4, 1, 5, 5, 1, 2, 5, 2, 6, 6, 2, 3, 6, 3, 7,
              3, 0, 7, 7, 0, 4, 4, 5, 7, 7, 5, 6};
            
            std::vector<Vector3> points_back = points;
            for (int j = 0; j < 8; ++j) {
              Real amt = (d / 2 + (j > 3 ? depth : 0));
              if (i == 2) {
                amt *= -1; // this was necessary because z axis is negative in, not sure, but fixed normals
              }
              points[j] += amt * depth_vec;
              points_back[j] -= amt * depth_vec;
              
              points[j] = shape->Transform() * points[j];
              points_back[j] = shape->Transform() * points_back[j];
            }
            
            
            builder.Add(points_back, indices);
            // Flip clockwise for normals
            std::reverse(indices.begin(), indices.end());
            builder.Add(points, indices);
            
            
          /*
            std::vector<Vector3> outer(4);
            outer[0] =
            
            Vector3 sizes = orig_sizes;
            sizes(0) -= inset;
            sizes(1) -= inset;
            sizes(2) -= inset;
            sizes(i) = depth;
            
            //sizes(i) = depth;
            
            Vector3 trans(0, 0, 0);
            trans(i) = orig_sizes(i) / 2 + depth / 2;
          
            Transform t1 = shape->Transform();
            t1.translation() += trans;
            
            Transform t2 = shape->Transform();
            t2.translation() -= trans;
            
            Box box1(sizes(0), sizes(1), sizes(2));
            builder.Add(box1, t1);
            builder.Add(box1, t2);
            */
          }
        }
          break;
        default:
          // Only boxes work for now
          assert(0);
      }
    }
    
    StlFormatWriter writer;
    
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
  float sim_time_;
  std::vector<boost::signals2::connection> conns_;
  Character *character_;
  JumpSolver jump_solver_;
  ActorAgent *debug_agent_;
};


} // namespace hsim

#endif // HSIM_PROJECT_HPP
