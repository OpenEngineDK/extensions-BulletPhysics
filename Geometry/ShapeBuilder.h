// Create bounding shapes from meshes
// -------------------------------------------------------------------
// Copyright (C) 2007 OpenEngine.dk (See AUTHORS) 
// 
// This program is free software; It is covered by the GNU General 
// Public License version 2 or any later version. 
// See the GNU General Public License for more details (see LICENSE). 
//--------------------------------------------------------------------

#ifndef _OE_SHAPE_BUILDER_H_
#define _OE_SHAPE_BUILDER_H_

#include <Math/Vector.h>

namespace OpenEngine {
    namespace Scene {
        class TransformationNode;
    }
namespace Geometry {

class AABBShape;
class Mesh;

class ShapeBuilder {
public:
    ShapeBuilder();
    virtual ~ShapeBuilder();
    
    AABBShape CreateAABBShape(Geometry::Mesh* mesh, Scene::TransformationNode* node = NULL);
};

} //NS Geometry
} //NS OpenEngine

#endif //_OE_SHAPE_BUILDER_H_
