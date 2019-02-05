//
//  Plans.hpp
//  hsimall
//
//  Created by z on 2/5/19.
//

#ifndef HSIM_PLANS_HPP
#define HSIM_PLANS_HPP

#include "hsim/Physics.hpp"

#include <iostream>

namespace hsim {

struct BoxPlanSorter {
  bool operator() (const AlignedBox& lhs, const AlignedBox& rhs) const
  {
    auto lhs_min = lhs.min();
    auto rhs_min = rhs.min();
    
    if (lhs_min[UpIdx] < rhs_min[UpIdx]) {
      return true;
    } else if (lhs_min[UpIdx] == rhs_min[UpIdx]) {
      const Real lhs_volume = lhs.volume();
      const Real rhs_volume = rhs.volume();
      if (lhs_volume < rhs_volume) {
        return true;
      } else if (lhs_volume == rhs_volume) {
        if (lhs_min[RtIdx] < rhs_min[RtIdx]) {
          return true;
        } else if (lhs_min[RtIdx] == rhs_min[RtIdx]) {
          if (lhs_min[InIdx] > rhs_min[InIdx]) {
            return true;
          }
        }
      }
    }
    
    return false;
  }
};

class PlanBuilder {
public:
  void Build(const RigidBody& body)
  {
    // Visit each shape
    std::vector<AlignedBox> boxes;
    auto& shapes = body.Shapes();
    for (std::size_t i = 0; i < shapes.size(); ++i) {
      auto& shape = shapes[i];
      Transform transform = body.Transform() * shape->Transform();
      AlignedBox box = TransformAlignedBox(
        shape->Geometry().BoundingBox(), transform);
      boxes.push_back(box);
    }
    
    Build(boxes);
    
    
  }
  
  void Build(const std::vector<AlignedBox>& boxes)
  {
    std::cout << "we have " << boxes.size() << " boxes." << std::endl;
    // First sort the boxes
    std::vector<AlignedBox> sorted_boxes = boxes;
    std::sort(sorted_boxes.begin(), sorted_boxes.end(), BoxPlanSorter());
    
    // Find the neighbors of each box
    for (std::size_t i = 0; i < sorted_boxes.size(); ++i) {
      // Write the box
      auto sizes = sorted_boxes[i].sizes();
      int itag = i + 1;
      std::cout << "Box " << itag << " (" << sizes[RtIdx] << " x " <<
        sizes[UpIdx] << " x " << sizes[InIdx] << ")\n";

      for (std::size_t j = 0; j < sorted_boxes.size(); ++j) {
        if (i == j) {
          continue;
        }
        
        if (sorted_boxes[i].squaredExteriorDistance(sorted_boxes[j]) < 0.0000001) {
          auto imin = sorted_boxes[i].min();
          auto imax = sorted_boxes[i].max();
          auto jmin = sorted_boxes[j].min();
          auto jmax = sorted_boxes[j].max();
          AlignedBox intersection = sorted_boxes[i].intersection(sorted_boxes[j]);
          
          if (jmin[RtIdx] >= imax[RtIdx]) {
            // To the Right
            std::cout << "Rt (" <<
              intersection.min()[InIdx] - imin[InIdx] << ", " <<
              imax[UpIdx] - intersection.max()[UpIdx] << " - " <<
              intersection.max()[InIdx] - imin[InIdx] << ", " <<
              imax[UpIdx] - intersection.min()[UpIdx] << ")\n";
          } else if (jmax[RtIdx] <= imin[RtIdx]) {
            // To the left
            std::cout << "Lt (" <<
              imax[InIdx] - intersection.max()[InIdx] << ", " <<
              imax[UpIdx] - intersection.max()[UpIdx] << " - " <<
              imax[InIdx] - intersection.min()[InIdx] << ", " <<
              imax[UpIdx] - intersection.min()[UpIdx] << ")\n";
          } else if (jmin[UpIdx] >= imax[UpIdx]) {
            // To the top
            std::cout << "Tp (" <<
              intersection.min()[RtIdx] - imin[RtIdx] << ", " <<
              imax[InIdx] - intersection.max()[InIdx] << " - " <<
              intersection.max()[RtIdx] - imin[RtIdx] << ", " <<
              imax[InIdx] - intersection.min()[InIdx] << ")\n";
          } else if (jmax[UpIdx] <= imin[UpIdx]) {
            // To the bottom
            std::cout << "Tp (" <<
              intersection.min()[RtIdx] - imin[RtIdx] << ", " <<
              intersection.min()[InIdx] - imin[InIdx] << " - " <<
              intersection.max()[RtIdx] - imin[RtIdx] << ", " <<
              intersection.max()[InIdx] - imin[InIdx] << ")\n";
          } else if (jmin[InIdx] >= imax[InIdx]) {
            // To the back
            std::cout << "Bk (" <<
              imax[RtIdx] - intersection.max()[RtIdx] << ", " <<
              imax[UpIdx] - intersection.max()[UpIdx] << " - " <<
              imax[RtIdx] - intersection.min()[RtIdx] << ", " <<
              imax[UpIdx] - intersection.min()[UpIdx] << ")\n";
          } else if (jmax[InIdx] <= imin[InIdx]) {
            std::cout << "Ft (" <<
              intersection.min()[RtIdx] - imin[RtIdx] << ", " <<
              imax[UpIdx] - intersection.max()[UpIdx] << " - " <<
              intersection.max()[RtIdx] - imin[RtIdx] << ", " <<
              imax[UpIdx] - intersection.min()[UpIdx] << ")\n";
          }
        }
      }
    }
    
    std::cout << std::endl;
  }

};


}

#endif /* HSIM_PLANS_HPP */
