// Axis Aligned Bounding Box.
// -------------------------------------------------------------------
// Copyright (C) 2007 OpenEngine.dk (See AUTHORS) 
// 
// This program is free software; It is covered by the GNU General 
// Public License version 2 or any later version. 
// See the GNU General Public License for more details (see LICENSE). 
//--------------------------------------------------------------------

#include <Geometry/AABBShape.h>

namespace OpenEngine {
namespace Geometry {



AABBShape::AABBShape() {

}

AABBShape::AABBShape(Vector<3,float> min, Vector<3,float> max)
    : min(min)
    , max(max)
{
    
}

AABBShape::~AABBShape() {

}

Vector<3,float> AABBShape::GetMin() const {
    return min;
}

Vector<3,float> AABBShape::GetMax() const {
    return max;
}

void AABBShape::SetMin(const Vector<3,float> min) {
    this->min = min;
}

void AABBShape::SetMax(const Vector<3,float> max) {
    this->max = max;
}

} //NS Geometry
} //NS OpenEngine
