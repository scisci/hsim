//
//  Project.cpp
//  hsim
//
//  Created by z on 1/12/19.
//

#include "hsim/Project.hpp"

namespace hsim {

std::unique_ptr<htree::Tree> Project::GenerateTree()
{
  std::uniform_int_distribution<int64_t> seed_dist(0, std::numeric_limits<int64_t>::max());
  htree::RatioSourcePtr ratio_source(new htree::golden::GoldenRatioSource());
  double container_ratio_xy = 0.5;
  double container_ratio_zy = 0.5;
  
  int num_leaves = std::uniform_int_distribution<>(200, 400)(rng);

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
  
  htree::EdgePathAttributer attributer(
    {{ .from = htree::EdgeName::kEdgeNameBottom, .to = htree::EdgeName::kEdgeNameTop },
    /*{ .from = htree::EdgeName::kEdgeNameBottom, .to = htree::EdgeName::kEdgeNameTop }*/
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

std::unique_ptr<Actor> Project::CreateActor(const htree::Tree& tree, const htree::StringNodeAttributes& attributes)
{
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
    
    builder.AddShape(std::unique_ptr<Geometry>(
      new Box(
        htree::AlignedBoxWidth(box),
        htree::AlignedBoxHeight(box),
        htree::AlignedBoxDepth(box))),
      density,
      transform);
    
    
  }
  
  return builder.Build();
}


} // namespace hsim
