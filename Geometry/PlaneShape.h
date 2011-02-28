// Analytic plane representation.
// -------------------------------------------------------------------
// Copyright (C) 2007 OpenEngine.dk (See AUTHORS) 
// 
// This program is free software; It is covered by the GNU General 
// Public License version 2 or any later version. 
// See the GNU General Public License for more details (see LICENSE). 
//--------------------------------------------------------------------

#ifndef _OE_PLANE_SHAPE_H_
#define _OE_PLANE_SHAPE_H_

#include <Geometry/ICollisionShape.h>
#include <Math/Vector.h>

namespace OpenEngine {
namespace Geometry {

using Math::Vector;

/**
 * Plane represented by a normal vector and a distance from origo.
 *
 *
 * @class PlaneShape PlaneShape.h Geometry/PlaneShape.h
 */
class PlaneShape : public ICollisionShape {
private:    
    Vector<3,float> norm;
    float dist;
public:
    PlaneShape();
    PlaneShape(Vector<3,float> norm, float dist);
    virtual ~PlaneShape();

    Vector<3,float> GetNormal() const;
    float GetDistance() const;

    void SetNormal(const Vector<3,float> norm);
    void SetDistance(const float dist);
};

} //NS Common
} //NS OpenEngine

#endif //_OE_PLANE_SHAPE_H_
