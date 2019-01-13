
#ifndef HSIM_PROJECT_HPP
#define HSIM_PROJECT_HPP

#include "hsim/Physics.hpp"

#include "htree/Golden.hpp"
#include "htree/RandomBasicGenerator.hpp"
#include "htree/RegionIterator.hpp"
#include "htree/EdgePathAttributer.hpp"

#include <memory>

namespace hsim {


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

} // namespace hsim

#endif // HSIM_PROJECT_HPP
