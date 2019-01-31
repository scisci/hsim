//
//  MotionPath.hpp
//  hsimall
//
//  Created by z on 1/28/19.
//

#ifndef HSIM_MOTION_PATH_HPP
#define HSIM_MOTION_PATH_HPP

#include "hsim/Math.hpp"

namespace hsim {

class MotionPath {
public:
  virtual ~MotionPath() {}
  
  virtual Real Length() const = 0;
  virtual Vector3 Compute(Real param) const = 0;
};

class ReversedMotionPath {
public:
  ReversedMotionPath(const MotionPath& path)
  : path_(&path),
    length_(path.Length())
  {}
  
  virtual Real Length() const
  {
    return length_;
  }
  
  virtual Vector3 Compute(Real param) const
  {
    return path_->Compute(length_ - param);
  }
  
private:
  const MotionPath *path_;
  Real length_;
};

class MotionPathCollisionDetector {
public:
  virtual ~MotionPathCollisionDetector() {}
  virtual bool CheckCollision(const Eigen::Ref<const Vector3>& pos) const = 0;
  virtual bool CheckCollision(const MotionPath& path) const = 0;
  
};

}

#endif /* HSIM_MOTION_PATH_HPP */
