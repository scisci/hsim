//
//  Complements.hpp
//  htreesim
//
//  Created by z on 11/6/18.
//

#ifndef HSIM_SIMULATION_HPP
#define HSIM_SIMULATION_HPP

namespace hsim
{

class Simulation
{
public:
  Simulation(const std::string &name);

  std::string Name();

private:
  std::string name_;
};

} // namespace hsim

#endif /* HSIM_SIMULATION_HPP */
