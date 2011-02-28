// Bullet debug drawer
// -------------------------------------------------------------------
// Copyright (C) 2007 OpenEngine.dk (See AUTHORS)
//
// This program is free software; It is covered by the GNU General
// Public License version 2 or any later version.
// See the GNU General Public License for more details (see LICENSE).
//--------------------------------------------------------------------

#ifndef _OE_BULLET_DEBUG_DRAWER_H_
#define _OE_BULLET_DEBUG_DRAWER_H_

#include <LinearMath/btIDebugDraw.h>
#include <Renderers/IRenderer.h>
#include <Core/IListener.h>

namespace OpenEngine {
    namespace Physics {
        class BulletPhysics;
    }
namespace Renderers {
    
class BulletDebugDrawer : public btIDebugDraw, public Core::IListener<RenderingEventArg> {
private:
    IRenderer* r;
    Physics::BulletPhysics* phys;
    int debugMode;
    Vector<3,float> begin, end;
public:
    BulletDebugDrawer(Physics::BulletPhysics* phys);
    virtual ~BulletDebugDrawer();
    
    void drawLine(const btVector3& from,const btVector3& to,const btVector3& color);
    void drawContactPoint(const btVector3& PointOnB, const btVector3& normalOnB, 
                          btScalar distance, int lifeTime, const btVector3& color);
    // void drawTriangle(const btVector3& v0, const btVector3& v1, const btVector3& v2,
    //                   const btVector3& n0, const btVector3& n1, const btVector3& n2,
    //                   const btVector3& color, btScalar alpha);
    // void drawTriangle(const btVector3& v0, const btVector3& v1,
    //                   const btVector3& v2, const btVector3& color, btScalar alpha);
    void reportErrorWarning(const char* warningString);
    void draw3dText(const btVector3& location, const char* textString);
    void setDebugMode(int debugMode);
    int getDebugMode() const;

    void Handle(RenderingEventArg arg);

    void SetRay(Vector<3,float> begin, Vector<3,float> end);
};

}
}

#endif //_OE_BULLET_DEBUG_DRAWER_H_
