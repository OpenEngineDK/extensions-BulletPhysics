// Traverse a scene and create rigid bodies from meshes.
// -------------------------------------------------------------------
// Copyright (C) 2007 OpenEngine.dk (See AUTHORS) 
// 
// This program is free software; It is covered by the GNU General 
// Public License version 2 or any later version. 
// See the GNU General Public License for more details (see LICENSE). 
//--------------------------------------------------------------------

#ifndef _OE_RIGID_BODY_BUILDER_H_
#define _OE_RIGID_BODY_BUILDER_H_

#include <Scene/ISceneNodeVisitor.h>
#include <Geometry/ShapeBuilder.h>
#include <Scene/SearchTool.h>

#include <stack>

namespace OpenEngine {
    namespace Scene {
        class ISceneNode;
        class MeshNode;
        class TransformationNode;
    }
namespace Physics {


class IPhysicsEngine;
class RigidBodyBuilder: public Scene::ISceneNodeVisitor {
private:    
    IPhysicsEngine& phys;
    Geometry::ShapeBuilder sb;
    Scene::SearchTool search;
    std::stack<Scene::TransformationNode*> tstack;
    Scene::ISceneNode* root;
public:
    RigidBodyBuilder(IPhysicsEngine& phys);
    virtual ~RigidBodyBuilder();
    
    Scene::ISceneNode* DoStuff(Scene::ISceneNode* node);
    void VisitMeshNode(Scene::MeshNode* node);
    void VisitTransformationNode(Scene::TransformationNode* node);
};

} //NS Geometry
} //NS OpenEngine

#endif //_OE_SHAPE_BUILDER_H_
