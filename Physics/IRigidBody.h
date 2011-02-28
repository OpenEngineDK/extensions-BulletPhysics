// Interface for a rigid body representation.
// -------------------------------------------------------------------
// Copyright (C) 2007 OpenEngine.dk (See AUTHORS)
//
// This program is free software; It is covered by the GNU General
// Public License version 2 or any later version.
// See the GNU General Public License for more details (see LICENSE).
//--------------------------------------------------------------------

#ifndef _OE_INTERFACE_RIGID_BODY_H_
#define _OE_INTERFACE_RIGID_BODY_H_

#include <Math/Vector.h>

namespace OpenEngine {
    namespace Scene {
        class TransformationNode;
    }
namespace Physics {

class IRigidBody {
public:
    virtual ~IRigidBody() {};

    virtual Scene::TransformationNode* GetTransformationNode() = 0;
    virtual void ApplyForce(const Math::Vector<3,float> force) = 0;
    virtual void ApplyForce(const Math::Vector<3,float> force, const Math::Vector<3,float> relPos) = 0;
    virtual void ApplyTorque(const Math::Vector<3,float> torque) = 0;

    virtual void SetLinearDamping(float damp) = 0;
    virtual void SetAngularDamping(float damp) = 0;
    virtual void SetMass(float mass) = 0;
};

}
}
#endif //_OE_INTERFACE_RIGID_BODY_H_
