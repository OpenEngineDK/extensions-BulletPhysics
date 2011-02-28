// Compound collision shape.
// -------------------------------------------------------------------
// Copyright (C) 2007 OpenEngine.dk (See AUTHORS) 
// 
// This program is free software; It is covered by the GNU General 
// Public License version 2 or any later version. 
// See the GNU General Public License for more details (see LICENSE). 
//--------------------------------------------------------------------

#include <Geometry/CompoundShape.h>

namespace OpenEngine {
namespace Geometry {

using namespace std;
 
CompoundShape::CompoundShape() {

}

CompoundShape::~CompoundShape() {

}

void CompoundShape::AddShape(ICollisionShape * shape) {
    shapes.push_back(shape);
}
    
vector<ICollisionShape*> CompoundShape::GetShapes() {
    return shapes;
}

}
}
