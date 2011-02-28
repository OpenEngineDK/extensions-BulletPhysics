// Axis Aligned Bounding Box.
// -------------------------------------------------------------------
// Copyright (C) 2007 OpenEngine.dk (See AUTHORS) 
// 
// This program is free software; It is covered by the GNU General 
// Public License version 2 or any later version. 
// See the GNU General Public License for more details (see LICENSE). 
//--------------------------------------------------------------------

#ifndef _OE_AXIS_ALIGNED_BOUNDING_BOX_H_
#define _OE_AXIS_ALIGNED_BOUNDING_BOX_H_

#include <Geometry/ICollisionShape.h>
#include <Math/Vector.h>

namespace OpenEngine {
namespace Geometry {

using Math::Vector;

/**
 * Axis aligned Bounding box for collision detection.
 *
 * Box is represented as two corners.
 *
 * @class AABBShape AABBShape.h Geometry/AABBShape.h
 */
class AABBShape : public ICollisionShape {

private:    
    Vector<3,float> min, max;
public:
    AABBShape();
    AABBShape(Vector<3,float> min, Vector<3,float> max);
    virtual ~AABBShape();

    Vector<3,float> GetMin() const;
    Vector<3,float> GetMax() const;
    void SetMin(const Vector<3,float> min);
    void SetMax(const Vector<3,float> max);
};

} //NS Common
} //NS OpenEngine

#endif
