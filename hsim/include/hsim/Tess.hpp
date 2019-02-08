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

#include <limits>

namespace hsim {


typedef std::vector<Vector3> TesselationVertexList;

struct Tesselation {
  std::vector<TesselationVertexList> objects;
};

class StlFormatWriter {
public:
  StlFormatWriter()
  : transform_(AffineTransform::Identity())
  {}
  
  void SetTransform(const AffineTransform& transform)
  {
    transform_ = transform;
  }
  
  void Write(const Tesselation& tesselation, std::ostream& out)
  {
    out.precision(std::numeric_limits<float>::max_digits10);
    out << std::scientific;
    
    out << "solid \n";
    
    for (auto& object : tesselation.objects) {
      for (size_t i = 0; i < object.size(); i+=3) {
        const Vector3 p1 = transform_ * object[i];
        const Vector3 p2 = transform_ * object[i + 1];
        const Vector3 p3 = transform_ * object[i + 2];
        // Calculate the normal using right handed cros product
        // we expect vertices to be counter clockwise
        const Vector3 normal = (p2 - p1).cross(p3 - p1).normalized();
        out << "facet normal " << normal.x() << " " << normal.y() << " " << normal.z() << "\n";
        out << "    outer loop\n";
        out << "        vertex " << p1.x() << " " << p1.y() << " " << p1.z() << "\n";
        out << "        vertex " << p2.x() << " " << p2.y() << " " << p2.z() << "\n";
        out << "        vertex " << p3.x() << " " << p3.y() << " " << p3.z() << "\n";
        out << "    endloop\n";
        out << "endfacet\n";
      }
    }
    
    out << "endsolid \n";
  }
  
private:
  AffineTransform transform_;
};

class ObjFormatWriter {
public:
  ObjFormatWriter()
  : transform_(AffineTransform::Identity())
  {}
  
  void SetTransform(const AffineTransform& transform)
  {
    transform_ = transform;
  }
  
  void Write(const Tesselation& tesselation, std::ostream& out)
  {
    out.precision(std::numeric_limits<float>::max_digits10);
    out << std::fixed;
    
    out << "g tess\n";
    
    for (auto& object : tesselation.objects) {
      for (size_t i = 0; i < object.size(); i+=3) {
        for (int j = i; j < i + 3; ++j) {
          const Vector3 vert = transform_ * object[j];
          out << "v " << vert.x() << " " << vert.y() << " " << vert.z() << "\n";
        }
        out << "f -3 -2 -1\n";
      }
    }
  }
  
private:
  AffineTransform transform_;
};

class TesselationBuilder {
public:
  TesselationBuilder(Handness handness)
  :handness_(handness)
  {}
  
  Handness Handness() const
  {
    return handness_;
  }
  
  void Clear()
  {
    tesselation_.objects.clear();
  }
  
  void Add(const std::vector<Vector3>& vertices, const std::vector<std::size_t> indices)
  {
    TesselationVertexList object(indices.size());
    for (size_t i = 0; i < object.size(); ++i) {
      object[i] = vertices[indices[i]];
    }
    
    tesselation_.objects.push_back(object);
  }
  
  
  

  
  void Add(const Box& box, const Transform& transform)
  {
    const Real half_width = box.Width() / 2.0;
    const Real half_height = box.Height() / 2.0;
    const Real half_depth = box.Depth() / 2.0;
    const Real zflip = handness_ == Handness::kRight ? 1 : -1;
    // Calculate vertices for front face and back face, clockwise start
    // top left.
    std::vector<Vector3> vertices = {
      transform * Vector3(-half_width, half_height,half_depth * zflip), // ftl
      transform * Vector3( half_width, half_height,half_depth * zflip), // ftr
      transform * Vector3( half_width,-half_height,half_depth * zflip), // fbr
      transform * Vector3(-half_width,-half_height,half_depth * zflip), // fbl
      transform * Vector3(-half_width, half_height,-half_depth * zflip), // btl
      transform * Vector3( half_width, half_height,-half_depth * zflip), // btr
      transform * Vector3( half_width,-half_height,-half_depth * zflip), // bbr
      transform * Vector3(-half_width,-half_height,-half_depth * zflip)};// bbl
    
    //   4 - - - 5
    //  /|      /|
    // 0 - - - 1 |
    // | 7 - - | 6
    // |/      |/
    // 3 - - - 2
    
    // counter clockwise
    std::vector<std::size_t> indices = {
      3, 1, 0, 3, 2, 1, // front
      7, 2, 3, 7, 6, 2, // bottom
      0, 5, 4, 0, 1, 5, // top
      6, 4, 5, 6, 7, 4, // back
      7, 0, 4, 7, 3, 0, // left
      2, 5, 1, 2, 6, 5};
    
    if (handness_ == Handness::kRight) {
      std::reverse(indices.begin(), indices.end());
    }

    // 6 Faces
    Add(vertices, indices);
  }
  
  const Tesselation& Tesselation() const
  {
    return tesselation_;
  }
  
private:
  struct Tesselation tesselation_;
  enum Handness handness_;
};

}

#endif // HSIM_TESS_HPP
