//
//  Complements.cpp
//  htreesim
//
//  Created by z on 11/6/18.
//

#include <cmath>
#include <string>

#include "hsim/Simulation.hpp"

namespace hsim
{

Simulation::Simulation(const std::string &name)
    : name_(name)
{
}

std::string Simulation::Name() const
{
  return name_;
}

} // namespace hsim
