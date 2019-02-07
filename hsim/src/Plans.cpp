//
//  Plans.cpp
//  FastXml
//
//  Created by z on 2/6/19.
//

#include "hsim/Plans.hpp"

namespace hsim {

std::ostream& operator<<(std::ostream &os, const FormattedValue& value)
{
    value.Write(os);
    return os;
}

std::string BoxSideName(BoxSide side)
{
  switch (side) {
    case BoxSide::kFront: return "Front";
    case BoxSide::kBack: return "Back";
    case BoxSide::kLeft: return "Left";
    case BoxSide::kRight: return "Right";
    case BoxSide::kTop: return "Top";
    case BoxSide::kBottom: return "Bottom";
    default: return "Unknown";
  }
}

BoxSide OppositeBoxSide(BoxSide side)
{
  switch (side) {
    case BoxSide::kFront: return BoxSide::kBack;
    case BoxSide::kBack: return BoxSide::kFront;
    case BoxSide::kLeft: return BoxSide::kRight;
    case BoxSide::kRight: return BoxSide::kLeft;
    case BoxSide::kTop: return BoxSide::kBottom;
    case BoxSide::kBottom: return BoxSide::kTop;
    default: return BoxSide::kFront;
  }
}

bool CompareBoxCut(const BoxCut& lhs, const BoxCut& rhs)
{
  if (lhs.side < rhs.side) {
    return true;
  }
  
  if (rhs.side < lhs.side) {
    return false;
  }
  
  if (lhs.min.x() < rhs.min.x()) {
    return true;
  }
  
  if (rhs.min.x() < lhs.min.x()) {
    return false;
  }
  
  if (lhs.min.y() <= rhs.min.y()) {
    return true;
  }
  
  return false;
}


}
