//
//  Project.cpp
//  hsim
//
//  Created by z on 1/12/19.
//

#include "hsim/Project.hpp"
#include "hsim/Colors.hpp"

namespace hsim {

std::unique_ptr<htree::Tree> Project::GenerateTree()
{
  std::uniform_int_distribution<int64_t> seed_dist(0, std::numeric_limits<int64_t>::max());
  htree::RatioSourcePtr ratio_source(new htree::golden::GoldenRatioSource());
  double container_ratio_xy = 0.5;
  double container_ratio_zy = 0.5;
  
  int num_leaves = std::uniform_int_distribution<>(100, 200)(rng); // 200 - 400

  htree::RandomBasicGenerator gen(ratio_source, container_ratio_xy, container_ratio_zy, num_leaves, seed_dist(rng));
  return gen.Generate();
}
  
htree::StringNodeAttributes Project::Attribute(const htree::Tree& tree)
{
  std::uniform_int_distribution<int64_t> seed_dist(0, std::numeric_limits<int64_t>::max());
  double attr_chaos = 1.0;
  
  htree::OverlapConstraint overlap;
  overlap.min_overlap_area = 0.01; // One cm - OR -
  overlap.min_overlap_norm = 0.01; // one percent
  
  htree::EdgePathAttributer2 attributer(
    {{ .from = htree::EdgeName::kEdgeNameBottom, .to = htree::EdgeName::kEdgeNameTop },
   /* { .from = htree::EdgeName::kEdgeNameLeft, .to = htree::EdgeName::kEdgeNameRight }*/
    },
    attr_chaos,
    seed_dist(rng),
    overlap);
  
  return attributer.Attribute(tree);
}
/*
BoxCluster BuildGeometry(const htree::Tree& tree, const htree::StringNodeAttributes& attributes)
{
  BoxCluster result;
 
  // Wood on Wood
  result.material = {
    .restitution = 0.6,
    .static_friction = 0.25,
    .kinetic_friction = 0.2
  };
 
  // Fir kg/m3
  result.density = 737.0;
 
  Real height_meters = 2.0;
  Real scale = height_meters / 1.0;
  htree::Vector origin(-scale / 2.0, -scale / 2.0, -scale / 2.0);
  htree::RegionIterator rit(tree, origin, scale);
 
  while (rit.HasNext()) {
    htree::NodeRegion node_region = rit.Next();
    if (node_region.node->Branch() != nullptr) {
      continue;
    }

    auto on_path = attributes.Attribute(node_region.node->ID(), "onPath");
    if (!on_path.second) {
      continue;
    }
 
    const htree::AlignedBox& box = node_region.region.AlignedBox();
    result.boxes.push_back(box);
  }
 
  return result;
}
*/

ActorContainer Project::CreateActor(const htree::Tree& tree, const htree::StringNodeAttributes& attributes, Handness handness)
{
  ActorContainer container;
  Real height_meters = 2.0;
  Real scale = height_meters / 1.0;
  htree::Vector origin(0.0, 0.0, 0.0);
  htree::RegionIterator rit(tree, origin, scale);
  
  hsim::RigidBodyBuilder builder;
  Real density = 737.0; // Fir kg/m3
  
  // Set to Wood on Wood
  builder.SetMaterial({
    .restitution = 0.6,
    .static_friction = 0.25,
    .kinetic_friction = 0.2});
  /*
  for (int i = 0; i < 4; i++) {
    Transform transform = Transform::Identity();
    transform.translation() = Vector3(1.0 * i,3.0, 0.0 * i);
    builder.AddShape(std::unique_ptr<Geometry>(
      new Box(
        0.5,
        0.5,
        1.0)),
      density,
      transform);
  }*/
  
  std::uniform_int_distribution<int64_t> color_dist(0, Colors::all.size() - 1);
  
  while (rit.HasNext()) {
    htree::NodeRegion node_region = rit.Next();
    if (node_region.node->Branch() != nullptr) {
      continue;
    }
    
    // TODO: filter out here
    auto result = attributes.Attribute(node_region.node->ID(), "onPath");
    if (!result.second) {
      continue;
    }
    
    const htree::AlignedBox& box = node_region.region.AlignedBox();
    Transform transform = Transform::Identity();
    transform.translation() = box.center();
    
    // Convert left handed to right handed
    if (handness == Handness::kRight) {
      transform.translation()[InIdx] = -transform.translation()[InIdx];
    }
    
    builder.AddShape(std::unique_ptr<Geometry>(
      new Box(
        htree::AlignedBoxWidth(box),
        htree::AlignedBoxHeight(box),
        htree::AlignedBoxDepth(box))),
      density,
      transform);
    
    
    container.color_map.push_back(Colors::all[color_dist(rng)]);
  }
  
  container.actor = builder.Build();
  return container;
  
}



std::unique_ptr<htree::Tree> CompositionProject::GenerateTree()
{
  std::uniform_int_distribution<int64_t> seed_dist(0, std::numeric_limits<int64_t>::max());
  std::shared_ptr<htree::golden::GoldenRatioSource> ratio_source(new htree::golden::GoldenRatioSource());
  
  htree::Complements complements = htree::MakeComplements(ratio_source->Core(), 0.0000001);
  auto usage = htree::FindRatioUsage(complements);
  for (auto& entry : usage) {
    std::cout << ratio_source->Ratios()[entry.index] << " : " << entry.count << std::endl;;
  }
  
  auto rm = htree::RatioGroup(ratio_source->Ratios());
  for (double a = -1.0; a < 20.0; a+= 0.01) {
    htree::ratio_index_t x = htree::FindClosestIndex(ratio_source->Ratios(), a);
    htree::ratio_index_t y = htree::FindClosestIndex(rm, a);
    assert(x == y);
  }

  htree::ratio_index_t index = htree::FindClosestIndex(ratio_source->Ratios(), 5.236);
  htree::TreeBuilder builder(ratio_source, index);
  double intro = 10.0;
  double ending = 28.0;
  double duration = 45.0;
  
  std::vector<double> positions = {intro / duration, ending / duration};
  bool result = htree::SectionLeaf(builder, complements, *builder.Leaves()[0], htree::Axis::X, positions, 0.0000001);
  assert(result);
  
  auto timestamps = SortByTimestamp(*builder.Build().get(), duration, 1000.0);
  
  for (int i = 0; i < 100; ++i) {
    htree::SplitRandomLeaf(builder, complements, rng, 0.0000001);
  }
  return builder.Build();
}
  
std::map<int64_t, std::vector<htree::NodeID>> SortByTimestamp(
  const htree::Tree &tree, double duration_secs, double scale)
{
  std::map<int64_t, std::vector<htree::NodeID>> timecodes;
  double sec_scale = duration_secs /
    tree.RatioSource()->Ratios()[tree.RatioIndexXY()];
  
  htree::RegionIterator rit(tree, htree::Vector(0, 0, 0), sec_scale);
  std::vector<htree::Region> regions;
  while (rit.HasNext()) {
    auto node = rit.Next();
    if (node.node->Branch() == nullptr) {
      int64_t timestamp = node.region.AlignedBox().min()[0] * scale + 0.5;
      timecodes[timestamp].push_back(node.node->ID());
    }
  }
  return timecodes;
}


ActorContainer CompositionProject::CreateActor(const htree::Tree& tree, Handness handness)
{
  ActorContainer container;
  Real height_meters = 2.0;
  Real scale = height_meters / 1.0;
  htree::Vector origin(0.0, 0.0, 0.0);
  htree::RegionIterator rit(tree, origin, scale);
  
  hsim::RigidBodyBuilder builder;
  Real density = 737.0; // Fir kg/m3
  
  // Set to Wood on Wood
  builder.SetMaterial({
    .restitution = 0.6,
    .static_friction = 0.25,
    .kinetic_friction = 0.2});
  /*
  for (int i = 0; i < 4; i++) {
    Transform transform = Transform::Identity();
    transform.translation() = Vector3(1.0 * i,3.0, 0.0 * i);
    builder.AddShape(std::unique_ptr<Geometry>(
      new Box(
        0.5,
        0.5,
        1.0)),
      density,
      transform);
  }*/
  
 
  std::uniform_int_distribution<int64_t> color_dist(0, Colors::all.size() - 1);
  std::unordered_map<htree::ratio_index_t, Color> cm;
  std::set<int64_t> used_colors;
  while (rit.HasNext()) {
    htree::NodeRegion node_region = rit.Next();
    if (node_region.node->Branch() != nullptr) {
      continue;
    }
    
    const htree::AlignedBox& box = node_region.region.AlignedBox();
    Transform transform = Transform::Identity();
    transform.translation() = box.center();
    
    // Convert left handed to right handed
    if (handness == Handness::kRight) {
      transform.translation()[InIdx] = -transform.translation()[InIdx];
    }
    
    // For the case of 2d
    Real depth = htree::AlignedBoxDepth(box);
    if (depth < 0.0000001) {
      assert(!htree::IsRatioIndexDefined(tree.RatioIndexZY()));
      depth = 1.0;
    }
    
    builder.AddShape(std::unique_ptr<Geometry>(
      new Box(
        htree::AlignedBoxWidth(box),
        htree::AlignedBoxHeight(box),
        depth)),
      density,
      transform);
    
    htree::ratio_index_t r = node_region.region.RatioIndexXY();
    auto it = cm.find(r);
    if (it == cm.end()) {
      std::size_t color_index = color_dist(rng);
      while (used_colors.find(color_index) != used_colors.end()) {
        color_index = color_dist(rng);
      }
      auto color = Colors::all[color_index];
      used_colors.insert(color_index);
      it = cm.insert(std::make_pair(r, color)).first;
    }
    container.color_map.push_back(it->second);
  }
  
  container.actor = builder.Build();
  return container;
  
}



} // namespace hsim
