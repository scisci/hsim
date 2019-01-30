//
//  ParabolicPlanner.hpp
//  hsimall
//
//  Created by z on 1/29/19.
//

#ifndef HSIM_PARABOLIC_PLANNER_H
#define HSIM_PARABOLIC_PLANNER_H

#include "hsim/Math.hpp"
#include "hsim/ParabolaMotionValidator.hpp"
#include "hsim/Collision.hpp"

#include <boost/graph/graph_traits.hpp>
#include <boost/graph/adjacency_list.hpp>
#include <boost/pending/disjoint_sets.hpp>
#include <boost/graph/incremental_components.hpp>

namespace hsim {

class ParabolicPlanner {
public:
  struct vertex_state_t
  {
      using kind = boost::vertex_property_tag;
  };
  
  struct edge_path_t
  {
      using kind = boost::edge_property_tag;
  };
  
  using Graph =
    boost::adjacency_list<boost::vecS, boost::vecS, boost::undirectedS,
      boost::property<vertex_state_t, Vector3,
        boost::property<boost::vertex_predecessor_t, unsigned long int,
          boost::property<boost::vertex_rank_t, unsigned long int>>>,
      boost::property<edge_path_t, std::shared_ptr<ParabolaPath>>>;
  
  using Vertex = boost::graph_traits<Graph>::vertex_descriptor;
  using Edge = boost::graph_traits<Graph>::edge_descriptor;
  
  using DisjointSets =
    boost::disjoint_sets<
      boost::property_map<Graph, boost::vertex_rank_t>::type,
      boost::property_map<Graph, boost::vertex_predecessor_t>::type>;
  
  ParabolicPlanner(
    Sampler* sampler,
    const ParabolaMotionValidator& motion_validator)
  : sampler_(sampler),
    motion_validator_(&motion_validator),
    state_prop_(boost::get(vertex_state_t(), graph_)),
    path_prop_(boost::get(edge_path_t(), graph_)),
    disjoint_sets_(
      boost::get(boost::vertex_rank, graph_),
      boost::get(boost::vertex_predecessor, graph_))
  {}
  
  void Solve(
    const Eigen::Ref<const Vector3>& start,
    const Eigen::Ref<const Vector3>& goal)
  {
    Vertex start_vertex = AddMilestone(start);
    Vertex goal_vertex = AddMilestone(goal);
    
    start_verts_.push_back(start_vertex);
    goal_verts_.push_back(goal_vertex);
    
    if (CheckSolution()) {
      std::cout << "Already connected, quitting early..." << std::endl;
      return;
    }
    
    for (int i = 0; i < 100; ++i) {
      auto result = sampler_->Sample();
      if (result.second) {
        AddMilestone(result.first);
        
        if (CheckSolution()) {
          std::cout << "Found solution, done..." << std::endl;
          return;
        }
      }
    }
    
    std::cout << "No solution" << std::endl;
  }

private:
  bool CheckSolution()
  {
    for (auto sit = start_verts_.begin(); sit != start_verts_.end(); ++sit) {
      for (auto git = goal_verts_.begin(); git != goal_verts_.end(); ++git) {
        if (boost::same_component(*sit, *git, disjoint_sets_)) {
          return true;
        }
      }
    }
    
    return false;
  }
  
  Vertex AddMilestone(const Eigen::Ref<const Vector3>& pos)
  {
    Vertex vert = boost::add_vertex(graph_);
    state_prop_[vert] = pos;
  
    disjoint_sets_.make_set(vert);
  
    for (auto neighbor : boost::make_iterator_range(vertices(graph_))) {
      if (neighbor == vert) {
        continue;
      }
      const Vector3& start = state_prop_[neighbor];
      const Vector3& end = state_prop_[vert];
      Vector3 normal(0, 1, 0);
      std::shared_ptr<ParabolaPath> path = motion_validator_->ComputePath(
        start, normal, end, normal);
      
      if (path != nullptr) {
        const Graph::edge_property_type properties(path);
        boost::add_edge(neighbor, vert, properties, graph_);
        disjoint_sets_.union_set(neighbor, vert);
      }
    }
    
    return vert;
  }
  
  Sampler *sampler_;
  const ParabolaMotionValidator *motion_validator_;
  
  Graph graph_;
  std::vector<Vertex> start_verts_;
  std::vector<Vertex> goal_verts_;
  
  boost::property_map<Graph, vertex_state_t>::type state_prop_;
  boost::property_map<Graph, edge_path_t>::type path_prop_;
  
  DisjointSets disjoint_sets_;
};

} // namespace hsim

#endif /* HSIM_PARABOLIC_PLANNER_H */
