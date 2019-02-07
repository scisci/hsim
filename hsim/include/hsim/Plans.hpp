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
#include <iomanip>

namespace hsim {

enum class PlanUnit {
  kInches,
  kCentimeters
};

struct FormattedValue {
  FormattedValue(Real value, PlanUnit unit)
  :value(value), unit(unit)
  {}
  
  //! Round to neares millimeter
  inline bool Write(std::ostream& out) const
  {
    Real value_units = ConvertToUnit(value, unit);
    if (!FormatUnit(value_units, unit, out)) {
      return false;
    }
    
    out << " " << UnitLabel(unit);
    return true;
  }
  
  inline std::string UnitLabel(PlanUnit unit) const
  {
    switch (unit) {
      case PlanUnit::kInches: return "in";
      case PlanUnit::kCentimeters: return "cm";
    }
    
    return "?";
  }
  
  inline Real ConvertToUnit(Real meters, PlanUnit unit) const
  {
    switch (unit) {
      case PlanUnit::kInches: return meters / 0.0254;
      case PlanUnit::kCentimeters: return meters / 0.01;
    }
    
    return meters;
  }
  
  inline bool FormatUnit(Real value, PlanUnit unit, std::ostream& out) const
  {
    switch (unit) {
      case PlanUnit::kInches:
        {
          int sixteenths = round(value * 16);
          int inches = sixteenths / 16;
          int remainder = sixteenths - inches * 16;
          if (remainder == 0) {
            out << inches;
            return true;
          }
          
          int num = remainder;
          int denom = 16;
          while ((num & 1) == 0) {
            num /= 2;
            denom /= 2;
          }
          
          out << inches << " " << num << "/" << denom;
          return true;
        }
        
      case PlanUnit::kCentimeters:
        std::streamsize ss = out.precision();
        out <<
          std::setprecision(3) <<
          (round(value * 1000) / 1000.0) <<
          std::setprecision(ss);
        return true;
    }
    
    return false;
  }

  Real value;
  PlanUnit unit;
};

std::ostream& operator<<(std::ostream &os, const FormattedValue& value);

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
      if (lhs_volume > rhs_volume) {
        return true;
      } else if (lhs_volume == rhs_volume) {
        if (lhs_min[RtIdx] < rhs_min[RtIdx]) {
          return true;
        } else if (lhs_min[RtIdx] == rhs_min[RtIdx]) {
          if ((handness_ == Handness::kRight && lhs.max()[InIdx] > rhs.max()[InIdx]) ||
              (handness_ == Handness::kLeft && lhs_min[InIdx] < rhs_min[InIdx])) {
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

enum class BoxSide {
  kFront,
  kBack,
  kLeft,
  kRight,
  kTop,
  kBottom
};

std::string BoxSideName(BoxSide side);

BoxSide OppositeBoxSide(BoxSide side);

struct BoxCut {
  BoxCut()
  :id(-1), side(BoxSide::kFront), min(Vector2::Zero()), max(Vector2::Zero())
  {}
  
  BoxCut(int id, BoxSide side, const Vector2Ref& min, const Vector2Ref& max)
  :id(id), side(side), min(min), max(max)
  {}
  
  int id;
  BoxSide side;
  Vector2 min;
  Vector2 max;
};

bool CompareBoxCut(const BoxCut& lhs, const BoxCut& rhs);

class PlanBuilder {
public:
  PlanBuilder(PlanUnit unit, Handness handness)
  :unit_(unit),
   handness_(handness)
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
    std::cout << boxes.size() << " boxes." << std::endl;
    // First sort the boxes
    std::vector<AlignedBox> sorted_boxes = boxes;
    std::sort(sorted_boxes.begin(), sorted_boxes.end(), BoxPlanSorter(handness_));
    
    // Find the neighbors of each box
    for (std::size_t i = 0; i < sorted_boxes.size(); ++i) {
      auto sizes = sorted_boxes[i].sizes();
      int itag = i + 1;
      std::cout << "\nBox " << itag << "\n" <<
        "------\n" <<
        Measurement(sizes[RtIdx]) << " wide\n" <<
        Measurement(sizes[UpIdx]) << " tall\n" <<
        Measurement(sizes[InIdx]) << " deep\n\n";
      
      std::vector<BoxCut> cuts;
      const Real epsilon = 1e-6;

      for (std::size_t j = 0; j < sorted_boxes.size(); ++j) {
        if (i == j) {
          continue;
        }

        if (sorted_boxes[i].squaredExteriorDistance(sorted_boxes[j]) < epsilon) {
          auto result = CalcNeighborCut(j, sorted_boxes[i], sorted_boxes[j], epsilon);
          if (result.second) {
            cuts.push_back(result.first);
          }
        }
      }
      
      std::sort(cuts.begin(), cuts.end(), CompareBoxCut);
      BoxSide side = BoxSide::kFront;
      bool first = true;
      for (const BoxCut& cut : cuts) {
        if (first || cut.side != side) {
          if (!first) {
            std::cout << "\n";
          } else {
            first = false;
          }
          side = cut.side;
          std::cout << "\t" << BoxSideName(side) << ":\n";
        }
        
        std::cout << "\t\tBox " << (cut.id + 1) << " (" <<
          BoxSideName(OppositeBoxSide(side)) << ")\n" <<
          "\t\t\t(" << Measurement(cut.min.x()) << ", " << Measurement(cut.min.y()) << ") bottom left\n" <<
          "\t\t\t(" << Measurement(cut.max.x()) << ", " << Measurement(cut.max.y()) << ") top right\n";
      }
    }
    
    std::cout << std::endl;
  }
  
private:
  //! Round to neares millimeter
  inline FormattedValue Measurement(Real value)
  {
    return FormattedValue(value, unit_);
  }
  
  
  std::pair<BoxCut, bool> CalcNeighborCut(
    int id,
    const AlignedBox& box,
    const AlignedBox& neighbor,
    Real epsilon) const
  {
    auto imin = box.min();
    auto imax = box.max();
    auto jmin = neighbor.min();
    auto jmax = neighbor.max();
    AlignedBox intersection = box.intersection(neighbor);
    
    auto sizes = intersection.sizes();
    
    // At least 2 planes must intersect to be a real neighbor, there are
    // planes that could be touching but at a diagonal that we filter out here
    int planes = 0;
    for (int i = 0; i < 3; ++i) {
      if (sizes[i] > epsilon) {
        planes++;
      }
    }
    
    if (planes < 2) {
      return std::make_pair(BoxCut(), false);
    }

    Transform transform = Transform::Identity();
    BoxSide side = BoxSide::kFront;
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

    //         TP
    //      4 - - - 5
    //     /|  BK  /|
    //    0 - - - 1 | RT
    // LT | 7 - - | 6
    //    |/  BT  |/
    //    3 - - - 2
    //        FR
    //
    // Take the face of intersection and find the transform that rotates
    // it onto the XY plane with the origin in the lower left. Then
    // project the intersection poitns onto the new space so we see
    // where to cut.

    const bool right_handed = handness_ == Handness::kRight;
    

    if (sizes[RtIdx] < epsilon) {
      if (jmin[RtIdx] >= imax[RtIdx] - epsilon) {
        side = BoxSide::kRight;
        const Real angle = right_handed ? -M_PI / 2 : M_PI / 2;
        const Real z = right_handed ? -imax[InIdx] : -imin[InIdx];
        transform = AngleAxis(angle, Vector3::Unit(UpIdx)) *
          Translation(-imax[RtIdx], -imin[UpIdx], z);
      } else if (imin[RtIdx] >= jmax[RtIdx] - epsilon) {
        side = BoxSide::kLeft;
        const Real angle = right_handed ? M_PI / 2 : -M_PI / 2;
        const Real z = right_handed ? -imin[InIdx] : -imax[InIdx];
        transform = AngleAxis(angle, Vector3::Unit(UpIdx)) *
          Translation(-imin[RtIdx], -imin[UpIdx], z);
      } else {
        assert(0);
      }
    } else if (sizes[UpIdx] < epsilon) {
      if (jmin[UpIdx] - imax[UpIdx] > -epsilon) {
        side = BoxSide::kTop;
        const Real angle = right_handed ? M_PI / 2 : -M_PI / 2;
        const Real z = right_handed ? -imax[InIdx] : -imin[InIdx];
        transform = AngleAxis(angle, Vector3::Unit(RtIdx)) *
          Translation(-imin[RtIdx], -imax[UpIdx], z);
      } else if (imin[UpIdx] - jmax[UpIdx] > -epsilon) {
        side = BoxSide::kBottom;
        const Real angle = right_handed ? -M_PI / 2 : M_PI / 2;
        const Real z = right_handed ? -imin[InIdx] : -imax[InIdx];
        transform = AngleAxis(angle, Vector3::Unit(RtIdx)) *
          Translation(-imin[RtIdx], -imin[UpIdx], z);
      } else {
        assert(0);
      }
    } else if (sizes[InIdx] < epsilon) {
      if ((right_handed && imin[InIdx] - jmax[InIdx] > -epsilon) ||
          (!right_handed && jmin[InIdx] - imax[InIdx] > -epsilon)) {
        side = BoxSide::kBack;
        const Real angle = M_PI;
        const Real z = right_handed ? -imin[InIdx] : -imax[InIdx];
        transform = AngleAxis(angle, Vector3::Unit(UpIdx)) *
          Translation(-imax[RtIdx], -imin[UpIdx], z);
      } else if ((right_handed && jmin[InIdx] - imax[InIdx] > -epsilon) ||
        (!right_handed && imin[InIdx] - jmax[InIdx] > -epsilon)) {
        side = BoxSide::kFront;
        const Real angle = 0;
        const Real z = right_handed ? -imin[InIdx] : -imax[InIdx];
        transform = AngleAxis(angle, Vector3::Unit(UpIdx)) *
          Translation(-imin[RtIdx], -imin[UpIdx], z);
      } else {
        assert(0);
      }
    } else {
      assert(0);
    }

    // Project corners onto xy plane
    AlignedBox projected;
    for (int corner = 0; corner < 8; ++corner) {
      Vector3 transformed = transform * corners[corner];
      projected.extend(transformed);
      /*
      std::cout << "(" << corners[corner].x() << ", " << corners[corner].y() << ", " << corners[corner].z() << ") becomes (" << transformed.x() << ", " << transformed.y() << ", " << transformed.z() << ")" << std::endl;
      */
    }

    return std::make_pair(
      BoxCut(
        id,
        side,
        Vector2(projected.min().x(), projected.min().y()),
        Vector2(projected.max().x(), projected.max().y())),
      true);
  }
  
  PlanUnit unit_;
  Handness handness_;
  
};


}

#endif /* HSIM_PLANS_HPP */
