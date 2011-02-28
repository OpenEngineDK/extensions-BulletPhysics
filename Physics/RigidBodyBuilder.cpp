// Traverse a scene and create rigid bodies from meshes.
// -------------------------------------------------------------------
// Copyright (C) 2007 OpenEngine.dk (See AUTHORS) 
// 
// This program is free software; It is covered by the GNU General 
// Public License version 2 or any later version. 
// See the GNU General Public License for more details (see LICENSE). 
//--------------------------------------------------------------------

#include <Physics/RigidBodyBuilder.h>
#include <Physics/IRigidBody.h>
#include <Physics/IPhysicsEngine.h>
#include <Geometry/ShapeBuilder.h>
#include <Geometry/ICollisionShape.h>
#include <Geometry/AABBShape.h>
#include <Scene/MeshNode.h>
#include <Scene/TransformationNode.h>
#include <Scene/SceneNode.h>
#include <Math/Quaternion.h>

namespace OpenEngine {
namespace Physics {

using namespace Geometry;
using namespace Scene;
using namespace Math;

RigidBodyBuilder::RigidBodyBuilder(IPhysicsEngine& phys)
    : phys(phys) {
    
}

RigidBodyBuilder::~RigidBodyBuilder() {
        
}
    
void RigidBodyBuilder::VisitMeshNode(MeshNode* node) {
    TransformationNode* t = tstack.top();

    AABBShape shape = sb.CreateAABBShape(node->GetMesh().get());
    IRigidBody* body = phys.CreateRigidBody(new AABBShape(shape), t);
    MeshNode* newNode = new MeshNode(node->GetMesh());
    if (t) {
        body->SetLinearDamping(0.99);
        body->SetAngularDamping(0.99);
        t->AddNode(newNode);
    }
    else {
        // static body
        body->SetMass(0.0);
        root->AddNode(newNode);
    }
    node->VisitSubNodes(*this);
}

void RigidBodyBuilder::VisitTransformationNode(Scene::TransformationNode* node) {
    TransformationNode* newNode = new TransformationNode();
    tstack.push(newNode);
    root->AddNode(newNode);
    TransformationNode* top = tstack.top();
    if (top) {
        Vector<3,float> p;
        Quaternion<float> q;
        top->GetAccumulatedTransformations(&p, &q);
        newNode->SetPosition(p);
        newNode->SetRotation(q);
    }
    node->VisitSubNodes(*this);
    tstack.pop();
}

ISceneNode* RigidBodyBuilder::DoStuff(ISceneNode* node) {
    // ISceneNode* parent = node->GetParent();
    // if (parent) parent->RemoveNode(node);

    root = new SceneNode();
    // root->AddNode(node);
    node->Accept(*this);
    // if (parent) parent->AddNode(node);
    // root->RemoveNode(node);
    return root;
}


} //NS Physics
} //NS OpenEngine
