// Analytic plane representation.
// -------------------------------------------------------------------
// Copyright (C) 2007 OpenEngine.dk (See AUTHORS) 
// 
// This program is free software; It is covered by the GNU General 
// Public License version 2 or any later version. 
// See the GNU General Public License for more details (see LICENSE). 
//--------------------------------------------------------------------

#include <Geometry/PlaneShape.h>

namespace OpenEngine {
namespace Geometry {



PlaneShape::PlaneShape()
        : dist(0.0) 
{

}

PlaneShape::PlaneShape(Vector<3,float> norm, float dist)
    : norm(norm)
    , dist(dist)
{
    
}

PlaneShape::~PlaneShape() {

}

Vector<3,float> PlaneShape::GetNormal() const {
    return norm;
}

float PlaneShape::GetDistance() const {
    return dist;
}

void PlaneShape::SetNormal(const Vector<3,float> min) {
    this->norm = norm;
}

void PlaneShape::SetDistance(const float dist) {
    this->dist = dist;
}

} //NS Geometry
} //NS OpenEngine
