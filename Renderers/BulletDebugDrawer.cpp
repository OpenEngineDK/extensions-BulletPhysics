// Bullet debug drawer
// -------------------------------------------------------------------
// Copyright (C) 2007 OpenEngine.dk (See AUTHORS)
//
// This program is free software; It is covered by the GNU General
// Public License version 2 or any later version.
// See the GNU General Public License for more details (see LICENSE).
//--------------------------------------------------------------------

#include <Renderers/BulletDebugDrawer.h>

#include <Physics/BulletPhysics.h>
#include <Logging/Logger.h>

namespace OpenEngine {
namespace Renderers {

using namespace Physics;

BulletDebugDrawer::BulletDebugDrawer(BulletPhysics* phys)
    : r(NULL)
    , phys(phys)
    , debugMode(1)
{

}

BulletDebugDrawer::~BulletDebugDrawer() {

}

void BulletDebugDrawer::drawLine(const btVector3& from,const btVector3& to,const btVector3& color) {
    Line l(BulletPhysics::ToOEVec(from), BulletPhysics::ToOEVec(to));
    r->DrawLine(l, BulletPhysics::ToOEVec(color));
}

void BulletDebugDrawer::SetRay(Vector<3,float> begin, Vector<3,float> end) {
    this->begin = begin;
    this->end = end;
}

// void BulletDebugDrawer::drawTriangle(const btVector3& v0,const btVector3& v1,const btVector3& v2,
//                           const btVector3& n0,const btVector3& n1,const btVector3& n2,
//                           const btVector3& color, btScalar alpha) {
  // p0 = BulletPhysics::ToOEVec(v0);
  // p1 = BulletPhysics::ToOEVec(v1);
  // p2 = BulletPhysics::ToOEVec(v2);
  // c = BulletPhysics::ToOEVec(color);
  // renderer->DrawLine(Line(p0,p1),c);
  // renderer->DrawLine(Line(p1,p2),c);
  // renderer->DrawLine(Line(p2,p0),c);
// }

// void BulletDebugDrawer::drawTriangle(const btVector3& v0,
//                                       const btVector3& v1,
//                                       const btVector3& v2,
//                                       const btVector3& color,
//                                       btScalar alpha) 
// {
  // p0 = toOEVec(v0);
  // p1 = toOEVec(v1);
  // p2 = toOEVec(v2);
  // c = toOEVec(color);
  // renderer->DrawLine(Line(p0,p1),c);
  // renderer->DrawLine(Line(p1,p2),c);
  // renderer->DrawLine(Line(p2,p0),c);
// }


void BulletDebugDrawer::setDebugMode(int debugMode) {
    this->debugMode = debugMode;
}

int BulletDebugDrawer::getDebugMode() const {
    return debugMode;
}

void BulletDebugDrawer::draw3dText(const btVector3& location,const char* textString) {

}

void BulletDebugDrawer::reportErrorWarning(const char* warningString) {
    logger.error << warningString << logger.end;
}

void BulletDebugDrawer::drawContactPoint(const btVector3& pointOnB,
                                         const btVector3& normalOnB,
                                         btScalar distance,
                                         int lifeTime,
                                         const btVector3& color) {
    // p1 = toOEVec(pointOnB+normalOnB*distance);
    // p0 = toOEVec(pointOnB);
    // c = toOEVec(color);
    // r->DrawLine(Line(p0,p1),c);
}

void BulletDebugDrawer::Handle(RenderingEventArg arg) {
    r = arg.canvas.GetRenderer();
    phys->world->setDebugDrawer(this);
    phys->world->debugDrawWorld();

    // Line l(begin, end);
    // r->DrawLine(l, Vector<3,float>(1.0,0.0,0.0));
    // r->DrawPoint(begin, Vector<3,float>(0.0,1.0,0.0), 4.0);
    // r->DrawPoint(end, Vector<3,float>(0.0,0.0,1.0), 4.0);
    
}

}
}
