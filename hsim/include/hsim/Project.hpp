
#ifndef HSIM_PROJECT_HPP
#define HSIM_PROJECT_HPP

#include "hsim/Physics.hpp"
#include "hsim/Colors.hpp"

#include "htree/Golden.hpp"
#include "htree/RandomBasicGenerator.hpp"
#include "htree/RegionIterator.hpp"
#include "htree/EdgePathAttributer.hpp"
#include "htree/TreeBuilder.hpp"

#include "hsim/Tess.hpp"


#include "hsim/JumpSolver.hpp"
#include "hsim/Plans.hpp"

#include <memory>
#include <iostream>
#include <fstream>

// Below is temporary only for DrawExtras code which will be reomved
#if PX_WINDOWS
#include <windows.h>
#pragma warning(disable: 4505)
#include <glut.h>
#elif PX_LINUX_FAMILY
#include <GL/glut.h>
#elif PX_OSX
#include <GLUT/glut.h>
#else
#error platform not supported.
#endif

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

struct ActorContainer {
  std::unique_ptr<Actor> actor;
  std::vector<Color> color_map;
};

class Project {
public:
  Project()
  :seed_(0),
   rng(seed_)
  {}
  
  virtual ~Project() {}
  
  void Seed(int64_t seed)
  {
    seed_ = seed;
    rng.seed(seed_);
  }
  
  int64_t LastSeed() const
  {
    return seed_;
  }
  std::unique_ptr<htree::Tree> GenerateTree();
  htree::StringNodeAttributes Attribute(const htree::Tree& tree);
  ActorContainer CreateActor(
    const htree::Tree& tree,
    const htree::StringNodeAttributes& attributes,
    Handness handness);
  
  
private:
  
  int64_t seed_;
  std::mt19937_64 rng;
};



class CompositionProject {
public:
  CompositionProject()
  :seed_(0),
   rng(seed_)
  {}
  
  virtual ~CompositionProject() {}
  
  void Seed(int64_t seed)
  {
    seed_ = seed;
    rng.seed(seed_);
  }
  
  int64_t LastSeed() const
  {
    return seed_;
  }
  std::unique_ptr<htree::Tree> GenerateTree();
  
  ActorContainer CreateActor(
    const htree::Tree& tree,
    Handness handness);
  
  
private:
  
  int64_t seed_;
  std::mt19937_64 rng;
};


enum IterationStatus {
  kIncomplete,
  kComplete,
  kFailed
};

class ProjectInstance : public RenderDecorator {
public:
  virtual ~ProjectInstance() {}
  virtual void Retry() = 0;
  virtual void Clear() = 0;
  virtual void Next() = 0;
  virtual void Open(const std::string& path) = 0;
  virtual void Write(const std::string& directory) = 0;
  virtual void DrawExtras() = 0;
  virtual IterationStatus Step() = 0;
};

class Iteration : public ProjectInstance {
public:
  Iteration(PhysicsEngine& engine)
  : simulation_(engine.CreateSimulation()),
    agent_(nullptr),
    state_(0),
    sim_time_(0.0),
    debug_agent_(nullptr),
    //right_(true),
    last_seed_(0),
    max_boxes_(1000)
  {
    //character_ = simulation_->AddCharacter(0.1, 0.5);
    //character_->SetPosition(Vector3(-10.0, 0.0, 0.0));
  }
  
  std::vector<Curve> curves;
  std::vector<hsim::Vector3> curve_verts;
  
  virtual void Retry()
  {
    Clear();
    Load();
  }
  
  virtual const std::vector<Color>* ActorColorMap(const Actor& actor) const
  {
    if (&actor == actor_.actor.get()) {
      return &actor_.color_map;
    }
    
    return nullptr;
  }
  
  virtual void Clear()
  {
    if (agent_ == nullptr) {
      return;
    }
    
    simulation_->RemoveActor(*agent_);
    
    if (debug_agent_ != nullptr) {
      simulation_->RemoveActor(*debug_agent_);
    }
    
    curves.clear();
    curve_verts.clear();

    for (auto& conn : conns_) {
      conn.disconnect();
    }
    conns_.clear();
    agent_ = nullptr;
    debug_agent_ = nullptr;
    state_ = 0;
    sim_time_ = 0;
  }
  
  virtual void Next()
  {
    Clear();
    BuildFromSeed(rd_());
  }
  
  void BuildFromSeed(int64_t seed)
  {
    std::cout << "BuildFromSeed " << seed << std::endl;
    last_seed_ = seed;
    project_.Seed(last_seed_);
    std::unique_ptr<htree::Tree> tree = project_.GenerateTree();
    htree::StringNodeAttributes attributes = project_.Attribute(*tree.get());
    
    Handness handness = Handness::kRight;
    actor_ = project_.CreateActor(*tree.get(), attributes, handness);
    
    Load();
    
   // Intersect();
  }
  
  void Load()
  {
    agent_ = simulation_->AddActor(*actor_.actor.get(), false);
    
    conns_.push_back(
      agent_->ConnectDidSleep(
        std::bind(&Iteration::HandleSleepCallback, this, agent_)));
  }
  
  // This is only to satisfy the fact that the structure is loose now, should
  // not have graphics code here
  virtual void DrawExtras()
  {
    for (auto& curve : curves) {
      glBegin(GL_LINE_STRIP);
    
      for (int i = curve.start_index; i < curve.end_index; ++i) {
        auto& vertex = curve_verts[i];
        glVertex3f(vertex.x(), vertex.y(), vertex.z());
      }
      glEnd();
    }
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
      RigidDynamic *rigid_actor = static_cast<RigidDynamic *>(actor_.actor.get());
      
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
        if (static_cast<const RigidBody&>(*actor_.actor.get()).Shapes().size() > max_boxes_) {
          state_ = 4;
        } else if (!Intersect()) {
          state_ = 4;
        }
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
    
    simulation_->AddActor(*result.get(), false);
  }
  
  bool Intersect()
  {
    if (actor_.actor->Type() != kRigidDynamic && actor_.actor->Type() != kRigidStatic) {
      return false;
    }
    
    const RigidBody& rigid_body = static_cast<const RigidBody&>(*actor_.actor.get());

    const Real cat_radius = 0.125;
    
    jump_solver_.Seed(last_seed_);
    const bool result = jump_solver_.Solve(rigid_body);
    
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
    debug_agent_ = simulation_->AddActor(*actor.get(), true);
    
    return result;
  }
  
  virtual IterationStatus Step()
  {
    float dt = 1.0f / 60.0f;
    sim_time_ += dt;
    simulation_->Step(dt);
    
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
  
  virtual void Open(const std::string& path)
  {
    std::ifstream proj_file;
    proj_file.open(path);
    
    int64_t seed;
    proj_file >> seed;

    Clear();
    BuildFromSeed(seed);
  }
  
  virtual void Write(const std::string& directory)
  {
    // Take our actor and tesselate and write it
    if (actor_.actor == nullptr) {
      return;
    }
    
    if (actor_.actor->Type() != kRigidDynamic && actor_.actor->Type() != kRigidStatic) {
      return;
    }
    
    const RigidActor& rigid_actor = static_cast<const RigidActor&>(*actor_.actor.get());
    
    TesselationBuilder builder(Handness::kRight);
    
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
          const bool bevel = true;
          if (!bevel) {
            builder.Add(box, shape->Transform());
          } else {
          
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
              // Outer square (clockwise)
              points[0] = -width/2 * width_vec - height/2 * height_vec;
              points[1] = width/2 * width_vec - height/2 * height_vec;
              points[2] = width/2 * width_vec + height/2 * height_vec;
              points[3] = -width/2 * width_vec + height/2 * height_vec;
              // Inner square (clockwise)
              points[4] = -(width/2 - inset) * width_vec - (height/2 - inset) * height_vec;
              points[5] = (width/2 - inset) * width_vec - (height/2 - inset) * height_vec;
              points[6] = (width/2 - inset) * width_vec + (height/2 - inset) * height_vec;
              points[7] = -(width/2 - inset) * width_vec + (height/2 - inset) * height_vec;
              
              std::vector<std::size_t> indices = {
                4, 1, 0, 5, 1, 4, // Top outside
                2, 1, 5, 6, 2, 5, // Right outside
                3, 2, 6, 7, 3, 6, // Bottom outside
                7, 0, 3, 4, 0, 7, // Left outside
                7, 5, 4, 6, 5, 7}; // inside
              
              std::vector<Vector3> points_back = points;
              for (int j = 0; j < 8; ++j) {
                Real amt = (d / 2 + (j > 3 ? depth : 0));
                
                // In a right handed system the z axis is negative towards the
                // back
                if (i == 2 && builder.Handness() == Handness::kRight) {
                  amt *= -1;
                }
                
                points[j] += amt * depth_vec;
                points_back[j] -= amt * depth_vec;
                
                points[j] = shape->Transform() * points[j];
                points_back[j] = shape->Transform() * points_back[j];
              }
              
              builder.Add(points, indices);
              // Flip vertices for the back points since they are specified from
              // the inside
              std::reverse(indices.begin(), indices.end());
              builder.Add(points_back, indices);
              
            }
          }
        }
          break;
        default:
          // Only boxes work for now
          assert(0);
      }
    }
    
    // First lets write the project which is just the seed
    std::ofstream proj_file;
    proj_file.open(directory + "/project.txt");
    proj_file << last_seed_;
    proj_file << std::endl;
    proj_file.close();
    
    // Second write the stl file
    std::ofstream stl_file;
    stl_file.open(directory + "/model.stl");
    
    StlFormatWriter writer;
    
    // 2m -> 12cm
    double scale = 12.0 / 200.0 * 1000.0; // Meters -> mm
    AffineTransform transform = AffineTransform::Identity();
    // Since we want z-axis up
    Eigen::AngleAxis<Real> axis_swap(0.5 * M_PI, Vector3::UnitX());
    transform.rotate(axis_swap);
    transform.scale(scale);
    writer.SetTransform(transform);
    writer.Write(builder.Tesselation(), stl_file);
    stl_file.close();
    
    
    // Write the build plan
    if (actor_.actor->Type() != kRigidDynamic && actor_.actor->Type() != kRigidStatic) {
      return;
    }
    
    const RigidBody& rigid_body = static_cast<const RigidBody&>(*actor_.actor.get());
    Real plan_scale = 0.5;
    std::ofstream plan_file;
    plan_file.open(directory + "/plan.txt");
    PlanBuilder pbuild(PlanUnit::kInches, plan_scale, Handness::kRight);
    pbuild.Write(plan_file, rigid_body, actor_.color_map);
    plan_file.close();
  }
  
private:
  std::unique_ptr<Simulation> simulation_;
  ActorContainer actor_;
  ActorAgent *agent_;
  int state_;
  float sim_time_;
  std::vector<boost::signals2::connection> conns_;
  Character *character_;
  JumpSolver jump_solver_;
  ActorAgent *debug_agent_;
  //bool right_;
  int64_t last_seed_;
  hsim::Project project_;
  std::random_device rd_;
  std::size_t max_boxes_;
};



struct Timecode {
  Timecode()
  :hours(0),
   minutes(0),
   seconds(0),
   millis(0)
  {}
  
  Timecode(double secs)
  {
    int total = secs * 1000 + 0.5;
    hours = total / 3600000;
    minutes = (total - hours * 3600000) / 60000;
    seconds = (total - hours * 3600000 - minutes * 60000) / 1000;
    millis = total % 1000;
  }
  
  Timecode(int64_t ms)
  :hours(ms / 3600000),
   minutes((ms - hours * 3600000) / 60000),
   seconds((ms - hours * 3600000 - minutes * 60000) / 1000),
   millis(ms % 1000)
  {}
  
  int hours;
  int minutes;
  int seconds;
  int millis;
};



struct TimecodeGroup {
  Timecode timecode;
  std::vector<htree::NodeID> ids;
};


std::map<int64_t, std::vector<htree::NodeID>> SortByTimestamp(
  const htree::Tree &tree, double duration_secs, double scale);


class CompositionDesigner : public ProjectInstance {
public:
  CompositionDesigner(PhysicsEngine& engine)
  : simulation_(engine.CreateSimulation()),
    agent_(nullptr),
    state_(0),
    sim_time_(0.0),
    debug_agent_(nullptr),
    last_seed_(0),
    max_boxes_(1000)
  {}

  virtual void Retry()
  {
    Clear();
    Load();
  }
  
  virtual const std::vector<Color>* ActorColorMap(const Actor& actor) const
  {
    if (&actor == actor_.actor.get()) {
      return &actor_.color_map;
    }
    
    return nullptr;
  }
  
  virtual void Clear()
  {
    if (agent_ == nullptr) {
      return;
    }
    
    simulation_->RemoveActor(*agent_);
    
    if (debug_agent_ != nullptr) {
      simulation_->RemoveActor(*debug_agent_);
    }
    
    agent_ = nullptr;
    debug_agent_ = nullptr;
    state_ = 0;
    sim_time_ = 0;
  }
  
  virtual void Next()
  {
    Clear();
    BuildFromSeed(rd_());
  }
  
  void BuildFromSeed(int64_t seed)
  {
    std::cout << "BuildFromSeed " << seed << std::endl;
    last_seed_ = seed;
    

    project_.Seed(last_seed_);
    std::unique_ptr<htree::Tree> tree = project_.GenerateTree();

    Handness handness = Handness::kRight;
    actor_ = project_.CreateActor(*tree.get(), handness);
    
    
    Load();
  }
  
  void Load()
  {
    agent_ = simulation_->AddActor(*actor_.actor.get(), false);
  }
  
  // This is only to satisfy the fact that the structure is loose now, should
  // not have graphics code here
  virtual void DrawExtras()
  {
  }
  
  
  
  virtual IterationStatus Step()
  {
    return kIncomplete;
  }
  
  virtual void Open(const std::string& path)
  {
    
    std::ifstream proj_file;
    proj_file.open(path);
    
    int64_t seed;
    proj_file >> seed;

    Clear();
    BuildFromSeed(seed);
    
  }
  
  virtual void Write(const std::string& directory)
  {
    // Take our actor and tesselate and write it
    if (actor_.actor == nullptr) {
      return;
    }
    
    if (actor_.actor->Type() != kRigidDynamic && actor_.actor->Type() != kRigidStatic) {
      return;
    }
    
    const RigidActor& rigid_actor = static_cast<const RigidActor&>(*actor_.actor.get());
    
    project_.Seed(last_seed_);
    std::unique_ptr<htree::Tree> tree = project_.GenerateTree();
    
    // First lets write the project which is just the seed
    std::ofstream proj_file;
    proj_file.open(directory + "/project.txt");
    proj_file << last_seed_;
    proj_file << std::endl;
    proj_file.close();
    
    auto tc = Timecode(37482.3232);
    assert(tc.hours == 10 && tc.minutes == 24 && tc.seconds == 42 && tc.millis == 323);
    tc = Timecode(61.300);
    assert(tc.hours == 0 && tc.minutes == 1 && tc.seconds == 1 && tc.millis == 300);
    tc = Timecode(59.9999);
    assert(tc.hours == 0 && tc.minutes == 1 && tc.seconds == 0 && tc.millis == 0);
    tc = Timecode(121.3882);
    assert(tc.hours == 0 && tc.minutes == 2 && tc.seconds == 1 && tc.millis == 388);
    tc = Timecode(121.3882);
    assert(tc.hours == 0 && tc.minutes == 2 && tc.seconds == 1 && tc.millis == 388);
    
    // Convert to milliseconds
    double duration = 45.0;
    auto timestamps = SortByTimestamp(*tree.get(), duration * 60.0, 1000.0);
    
    // Write each timestamp to csv
    std::ofstream csv_file;
    csv_file.open(directory + "/timestamps.csv");
    csv_file << "Scene Number,Timecode,Node Id\n";
    int scene_num = 1;
    for (const auto &entry : timestamps) {
      csv_file << (scene_num++) << ",";
      
     
      Timecode tc(entry.first);
      csv_file << std::setfill('0') << std::setw(2) << tc.hours << ":";
      csv_file << std::setfill('0') << std::setw(2) << tc.minutes << ":";
      csv_file << std::setfill('0') << std::setw(2) << tc.seconds << ".";
      csv_file << std::setfill('0') << std::setw(3) << tc.millis;
      csv_file << ",";
      
       bool first = true;
      for (auto &id : entry.second) {
        if (first) {
          first = false;
        } else {
          csv_file << "+";
        }
        csv_file << id;
      }
      csv_file << "\n";
      
    }
    csv_file.close();
    
    double ratio = tree->RatioSource()->Ratios()[tree->RatioIndexXY()];
    double psscale = 100.0;
    double width = psscale * ratio;
    double height = 1 * psscale;
    double border = 2.0;
    double top_border = 50.0;
    std::ofstream ps_file;
    ps_file.open(directory + "/map.eps");
    ps_file << "%%!PS-Adobe-3.0 EPSF-3.0\n";
    ps_file << "%%Creator: Dan Riley\n";
    ps_file << "%%BoundingBox: " << -border << " " << -border << " " << (width + border * 2) << " " << (height + top_border + border * 2) << "\n";

    ps_file << "0.5 setlinewidth\n";
    ps_file << "0 0 0 setrgbcolor\n";
    htree::RegionIterator rit(*tree.get(), htree::Vector(0.0, 0.0, 0), psscale);
    while (rit.HasNext()) {
      auto rn = rit.Next();
      if (rn.node->Branch() == nullptr) {
        const htree::AlignedBox &box = rn.region.AlignedBox();
        htree::Vector min = box.min();
        htree::Vector max = box.max();
        ps_file << min.x() << " " << min.y() << " newpath moveto\n";
        ps_file << min.x() << " " << max.y() << " lineto\n";
        ps_file << max.x() << " " << max.y() << " lineto\n";
        ps_file << max.x() << " " << min.y() << " lineto\n";
        ps_file << min.x() << " " << min.y() << " lineto\n";
        ps_file << "stroke\n";
      }
    }
    
    ps_file << "0 0 1 setrgbcolor\n";
    rit = htree::RegionIterator (*tree.get(), htree::Vector(0.0, 0.0, 0), psscale);
    while (rit.HasNext()) {
      auto rn = rit.Next();
      if (rn.node->Branch() == nullptr) {
        const htree::AlignedBox &box = rn.region.AlignedBox();
        htree::Vector min = box.min();
        htree::Vector max = box.max();
        ps_file << "/Arial findfont\n3 scalefont\nsetfont\nnewpath\n";
        ps_file << box.center().x() << " " << box.center().y() << " moveto ";
        ps_file << "(" << rn.node->ID() << "-" << rn.region.RatioIndexXY() << ") dup stringwidth pop 2 div neg 0 rmoveto show\n";
      }
    }
    
    // Draw on scene numbers
    ps_file << "0 0 0 setrgbcolor\n";
    scene_num = 1;
    for (const auto &entry : timestamps) {
      Timecode tc(entry.first);
      double pos = (entry.first * ratio * psscale) / (duration * 60.0 * 1000.0);
      double line_start = psscale + 2.0;
      double line_end = psscale + 2.0 + (top_border - 2.0) * ((12 - scene_num % 12) / 12.0);
      ps_file << pos << " " << line_start << " newpath moveto\n";
      ps_file << pos << " " << line_end << " lineto\n";
      ps_file << "stroke\n";
      scene_num++;
    }
    
    ps_file << "1 0 0 setrgbcolor\n";
    scene_num = 1;
    for (const auto &entry : timestamps) {
      Timecode tc(entry.first);
      double pos = (entry.first * ratio * psscale) / (duration * 60.0 * 1000.0);
      double line_start = psscale + 2.0;
      double line_end = psscale + 2.0 + (top_border - 2.0) * ((12 - scene_num % 12) / 12.0);
      
      ps_file << "/Arial findfont\n3 scalefont\nsetfont\nnewpath\n";
      ps_file << (pos + 2) << " " << (line_end - 2) << " moveto ";
      ps_file << "(" << scene_num << " ";
      ps_file << std::setfill('0') << std::setw(2) << tc.hours << ":";
      ps_file << std::setfill('0') << std::setw(2) << tc.minutes << ":";
      ps_file << std::setfill('0') << std::setw(2) << tc.seconds;
      ps_file << ") show\n";
      scene_num++;
    }
    
    ps_file << "\n\n";
    ps_file << "%%EOF\n";
    ps_file.close();
  
    
  }
  
private:
  std::unique_ptr<Simulation> simulation_;
  ActorContainer actor_;
  hsim::CompositionProject project_;
  ActorAgent *agent_;
  int state_;
  float sim_time_;
  ActorAgent *debug_agent_;
  //bool right_;
  int64_t last_seed_;
  std::random_device rd_;
  std::size_t max_boxes_;
};



} // namespace hsim

#endif // HSIM_PROJECT_HPP
