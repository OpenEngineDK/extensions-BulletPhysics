// Basic movement handler.
// -------------------------------------------------------------------
// Copyright (C) 2007 OpenEngine.dk (See AUTHORS) 
// 
// This program is free software; It is covered by the GNU General 
// Public License version 2 or any later version. 
// See the GNU General Public License for more details (see LICENSE). 
//--------------------------------------------------------------------

#include <Utils/ForceHandler.h>

#include <Physics/IPhysicsEngine.h>
#include <Renderers/BulletDebugDrawer.h>
#include <Physics/IRigidBody.h>
#include <Display/IViewingVolume.h>
#include <Display/IFrame.h>
#include <Geometry/Ray.h>

#include <Logging/Logger.h>

namespace OpenEngine {
namespace Utils {

using namespace Physics;
using namespace Display;
using namespace Devices;
using namespace Geometry;
using namespace Renderers;

ForceHandler::ForceHandler(IPhysicsEngine& phys, 
                           IViewingVolume& vv, 
                           IFrame& frame, 
                           IMouse& mouse
                           // BulletDebugDrawer* dbd
                           )
    : phys(phys)
    , vv(vv) 
    , frame(frame)
    , mouse(mouse)
    , body(NULL)
    // , dbd(dbd)
{

}

ForceHandler::~ForceHandler() {

}

Vector<3,float> ForceHandler::ScreenToWorld(float x, float y, float d) {
    float w = frame.GetWidth();
    float h = frame.GetHeight();

    Matrix<4,4,float> proj = vv.GetProjectionMatrix();
    proj.Transpose();
    Matrix<4,4,float> view = vv.GetViewMatrix();
    view.Transpose();
    Matrix<4,4,float> inv = (proj * view).GetInverse();
    
    Vector<4,float> pos4((x / w)*2.0 - 1.0, ((h - y) / h) * 2.0 - 1.0, d, 1.0);

    pos4 = inv * pos4;

    return Vector<3,float>(pos4[0]/pos4[3], 
                           pos4[1]/pos4[3], 
                           pos4[2]/pos4[3]);
}


Vector<4,float> ForceHandler::Unproj(float x, float y, float d) {
    float w = frame.GetWidth();
    float h = frame.GetHeight();
 
    Matrix<4,4,float> invProj = vv.GetProjectionMatrix().GetInverse();
    Vector<4,float> pos4((x / w) * 2.0 - 1.0, ((h - y) / h) * 2.0 - 1.0, d, 1.0);

    logger.info << "screen space: " << pos4 << logger.end;
    // logger.info << "invProj: " << invProj << logger.end;
    
    pos4 = invProj * pos4;
    pos4 *= pos4[3];
    logger.info << "view space: " << pos4 << logger.end;

    return Vector<4,float>(pos4[0], 
                           pos4[1], 
                           pos4[2],
                           1.0);
}

Vector<3,float> ForceHandler::ViewToWorld(Vector<4, float> v) {
    Matrix<4,4,float> invView = vv.GetViewMatrix().GetInverse();
    Vector<4,float> v1 = invView * v;
    return Vector<3,float>(v1[0], v1[1], v1[2]);
} 

void ForceHandler::Handle(MouseMovedEventArg arg) {
    if (body) {
        Vector<3,float> newPos = ScreenToWorld(arg.x, arg.y, 0.0);
        Vector<3,float> force = (newPos - pos) * 10000.0;

        // logger.info << "apply force: " << force << " to: " << body << logger.end;

        body->ApplyForce(force, relPos);
        pos = newPos;
    }
}
    
void ForceHandler::Handle(MouseButtonEventArg arg) {
    if (arg.button == BUTTON_RIGHT) {
        if (arg.type == EVENT_PRESS) {
            restorePos[0] = arg.state.x;
            restorePos[1] = arg.state.y;
            
            Vector<3,float> begin = ScreenToWorld(arg.state.x, arg.state.y, 0.0);
            Vector<3,float> end = ScreenToWorld(arg.state.x, arg.state.y, 1.0);
            body = phys.CastRay(begin, end, &relPos);
            // dbd->SetRay(begin, end);
            if (body) {
                mouse.HideCursor();
                pos = begin;
                // logger.info << "body: " << body << logger.end;
            }
        }
        else if ((arg.type == EVENT_RELEASE) && body) {
            body = NULL;
            mouse.SetCursor(restorePos[0], restorePos[1]);
            mouse.ShowCursor();
        }
    }
}
    

} // NS Utils
} // NS OpenEngine
