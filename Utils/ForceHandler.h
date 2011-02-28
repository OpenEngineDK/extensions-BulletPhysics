// 
// -------------------------------------------------------------------
// Copyright (C) 2007 OpenEngine.dk (See AUTHORS)
//
// This program is free software; It is covered by the GNU General
// Public License version 2 or any later version.
// See the GNU General Public License for more details (see LICENSE).
//--------------------------------------------------------------------


#ifndef _OE_FORCE_HANDLER_H_
#define _OE_FORCE_HANDLER_H_

#include <Devices/IMouse.h>
#include <Core/IListener.h>
#include <Math/Vector.h>

namespace OpenEngine {
    namespace Physics {
        class IPhysicsEngine;
        class IRigidBody;
    }
    namespace Display {
        class IViewingVolume;
        class IFrame;
    }
    namespace Renderers{
        class BulletDebugDrawer;
    }
namespace Utils {

class ForceHandler 
    : public Core::IListener<Devices::MouseMovedEventArg>,
      public Core::IListener<Devices::MouseButtonEventArg> {
private:
    Physics::IPhysicsEngine& phys;
    Display::IViewingVolume& vv;
    Display::IFrame& frame;
    Devices::IMouse& mouse;
    Physics::IRigidBody* body;
    Math::Vector<3,float> pos;
    Math::Vector<3,float> relPos;
    Math::Vector<2,int> restorePos;
    
    Renderers::BulletDebugDrawer* dbd;
    inline Math::Vector<4,float> Unproj(float x, float y, float d);
    inline Math::Vector<3,float> ScreenToWorld(float x, float y, float d);
    inline Math::Vector<3,float> ViewToWorld(Math::Vector<4,float> v);
public:
    ForceHandler(Physics::IPhysicsEngine& phys, 
                 Display::IViewingVolume& vv,
                 Display::IFrame& frame,
                 Devices::IMouse& mouse
                 // Renderers::BulletDebugDrawer* dbd = NULL
                 );
    virtual ~ForceHandler();
    void Handle(Devices::MouseMovedEventArg arg);
    void Handle(Devices::MouseButtonEventArg arg);
};

} // NS Utils
} // NS OpenEngine

#endif // _OE_FORCE_HANDLER_H_
