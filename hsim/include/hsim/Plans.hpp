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
  BoxPlanSorter(Handness handness)
  : handness_(handness)
  {}
  
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
          if ((handness_ == Handness::kRight && lhs.max()[InIdx] > rhs.max()[InIdx]) ||
              (handness_ == Handness::kLeft && lhs_min[InIdx] > rhs_min[InIdx])) {
            return true;
          }
        }
      }
    }
    
    return false;
  }
  
private:
  Handness handness_;
};

class PlanBuilder {
public:
  PlanBuilder(Handness handness)
  :handness_(handness)
  {}
  
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
    std::sort(sorted_boxes.begin(), sorted_boxes.end(), BoxPlanSorter(handness_));
    
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
          
          Transform transform = Transform::Identity();
          Vector3 corners[] = {
            intersection.corner(AlignedBox::CornerType::TopLeftFloor),
            intersection.corner(AlignedBox::CornerType::TopRightFloor),
            intersection.corner(AlignedBox::CornerType::BottomRightFloor),
            intersection.corner(AlignedBox::CornerType::BottomLeftFloor),
            intersection.corner(AlignedBox::CornerType::TopLeftCeil),
            intersection.corner(AlignedBox::CornerType::TopRightCeil),
            intersection.corner(AlignedBox::CornerType::BottomRightCeil),
            intersection.corner(AlignedBox::CornerType::BottomLeftCeil)
          };
          
          if (jmin[RtIdx] >= imax[RtIdx]) {
            // To the Right
            transform = Eigen::AngleAxis<Real>(-M_PI / 2, Vector3::Unit(UpIdx)) * Eigen::Translation<Real, 3>(-imax[RtIdx], -imin[UpIdx], -imax[InIdx]);
          } else if (jmax[RtIdx] <= imin[RtIdx]) {
            // To the left
            transform = Eigen::AngleAxis<Real>(M_PI / 2, Vector3::Unit(UpIdx)) * Eigen::Translation<Real, 3>(-imin[RtIdx], -imin[UpIdx], -imin[InIdx]);
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
          
          // Project corners onto xy plane
          AlignedBox projected;
          for (int corner = 0; corner < 8; ++corner) {
            Vector3 transformed = transform * corners[corner];
            projected.extend(transformed);
            std::cout << "(" << corners[corner].x() << ", " << corners[corner].y() << ", " << corners[corner].z() << ") becomes (" << transformed.x() << ", " << transformed.y() << ", " << transformed.z() << ")" << std::endl;
          }
          
          std::cout << "(" <<
            projected.min().x() << ", " << projected.min().y() << ") - (" <<
            projected.max().x() << ", " << projected.max().y() << ")\n" ;
        }
      }
    }
    
    std::cout << std::endl;
  }
  
private:
  Handness handness_;

};


}

#endif /* HSIM_PLANS_HPP */
