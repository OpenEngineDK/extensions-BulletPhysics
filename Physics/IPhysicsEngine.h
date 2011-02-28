// General interface for a physics engine.
// -------------------------------------------------------------------
// Copyright (C) 2007 OpenEngine.dk (See AUTHORS)
//
// This program is free software; It is covered by the GNU General
// Public License version 2 or any later version.
// See the GNU General Public License for more details (see LICENSE).
//--------------------------------------------------------------------

#ifndef _OE_INTERFACE_PHYSICS_ENGINE_H_
#define _OE_INTERFACE_PHYSICS_ENGINE_H_

#include <Core/IModule.h>
#include <Math/Vector.h>

namespace OpenEngine {
    namespace Scene {
        class TransformationNode;
    }
    namespace Geometry {
        class ICollisionShape;
    }
    namespace Physics {

class IRigidBody;

class IPhysicsEngine: public Core::IModule {
public:
    virtual ~IPhysicsEngine() {};

    virtual void SetGravity(const Math::Vector<3,float> g) = 0;
    virtual Math::Vector<3,float> GetGravity() const = 0;

    virtual IRigidBody* CreateRigidBody(Geometry::ICollisionShape* shape, 
                                        Scene::TransformationNode* node = NULL) = 0;

    virtual void AddShape(Geometry::ICollisionShape* shape) = 0;

    virtual IRigidBody* CastRay(Math::Vector<3,float> begin, Math::Vector<3,float> end, Math::Vector<3,float>* hitPoint = NULL) = 0;
};

}
}

#endif //_OE_INTERFACE_PHYSICS_ENGINE_H_
