//
//  Tess.hpp
//  hsimall
//
//  Created by z on 1/14/19.
//

#ifndef HSIM_TESS_HPP
#define HSIM_TESS_HPP

#include "hsim/Math.hpp"

#include "htree/Geometry.hpp"

namespace hsim {


typedef std::vector<Vector3> TesselationVertexList;

struct Tesselation {
  std::vector<TesselationVertexList> objects;
};

class TesselationBuilder {
public:
  void Add(
};

}

#endif // HSIM_TESS_HPP
