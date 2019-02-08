//
//  Colors.hpp
//  hsimall
//
//  Created by z on 2/8/19.
//

#ifndef HSIM_COLORS_HPP
#define HSIM_COLORS_HPP

#include <string>
#include <vector>

namespace hsim {

struct Color {
  Color(const std::string& name, int value)
  :name(name), value(value)
  {}
  
  std::string name;
  int value;
};

class Colors {
public:
  static const std::vector<Color> all;
};

}

#endif /* Colors_h */
