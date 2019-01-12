//
//  Scene.hpp
//  hsimall
//
//  Created by z on 1/11/19.
//

#ifndef HSIM_SCENE_HPP
#define HSIM_SCENE_HPP

#include "hsim/Models.hpp"

namespace hsim {

class Scene {
public:
  virtual ~Scene() {}
  //virtual void AddBoxCluster(const BoxCluster& box_cluster) = 0;
};

}

#endif // HSIM_SCENE_HPP
