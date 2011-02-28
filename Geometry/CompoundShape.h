// Compound collision shape.
// -------------------------------------------------------------------
// Copyright (C) 2007 OpenEngine.dk (See AUTHORS) 
// 
// This program is free software; It is covered by the GNU General 
// Public License version 2 or any later version. 
// See the GNU General Public License for more details (see LICENSE). 
//--------------------------------------------------------------------

#ifndef _OE_COMPOUND_SHAPE_H_
#define _OE_COMPOUND_SHAPE_H_

#include <Geometry/ICollisionShape.h>

#include <vector>

namespace OpenEngine {
    namespace Scene {
        class TransformationNode;
    }
namespace Geometry {

using Scene::TransformationNode;

class CompoundShape : public ICollisionShape {
private:
    std::vector<ICollisionShape*> shapes;
public:

    CompoundShape();
    ~CompoundShape();
    
    void AddShape(ICollisionShape * shape);
    std::vector<ICollisionShape*> GetShapes();
};

}
}

#endif //_OE_COMPOUND_SHAPE_H_
