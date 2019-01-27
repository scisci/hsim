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
        // Calculate the normal
        const Vector3 normal = (object[i + 1] - object[i]).cross(object[i+2] - object[i]).normalized();
        out << "facet normal " << normal.x() << " " << normal.y() << " " << normal.z() << "\n";
        out << "    outer loop\n";
        for (int j = i; j < i + 3; ++j) {
          const Vector3 vert = transform_ * object[j];
          out << "        vertex " << vert.x() << " " << vert.y() << " " << vert.z() << "\n";
        }
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
    
    // Calculate vertices for front face and back face, clockwise start
    // top left.
    std::vector<Vector3> vertices = {
      transform * Vector3(-half_width, half_height,-half_depth), // ftl
      transform * Vector3( half_width, half_height,-half_depth), // ftr
      transform * Vector3( half_width,-half_height,-half_depth), // fbr
      transform * Vector3(-half_width,-half_height,-half_depth), // fbl
      transform * Vector3(-half_width, half_height, half_depth), // btl
      transform * Vector3( half_width, half_height, half_depth), // btr
      transform * Vector3( half_width,-half_height, half_depth), // bbr
      transform * Vector3(-half_width,-half_height, half_depth)};// bbl
    
    //   4 - - - 5
    //  /|      /|
    // 0 - - - 1 |
    // | 7 - - | 6
    // |/      |/
    // 3 - - - 2
    
    std::vector<std::size_t> indices = {
      0, 1, 3, 1, 2, 3, // front
      3, 2, 7, 2, 6, 7, // bottom
      4, 5, 0, 5, 1, 0, // top
      5, 4, 6, 4, 7, 6, // back
      4, 0, 7, 0, 3, 7, // left
      1, 5, 2, 5, 6, 2};
    
    // 6 Faces
    Add(vertices, indices);
  }
  
  const Tesselation& Tesselation() const
  {
    return tesselation_;
  }
  
private:
  struct Tesselation tesselation_;
};

}

#endif // HSIM_TESS_HPP
