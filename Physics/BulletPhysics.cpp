// Concrete implementation of a physics engine using bullet.
// -------------------------------------------------------------------
// Copyright (C) 2007 OpenEngine.dk (See AUTHORS)
//
// This program is free software; It is covered by the GNU General
// Public License version 2 or any later version.
// See the GNU General Public License for more details (see LICENSE).
//--------------------------------------------------------------------

#include <Physics/BulletPhysics.h>
#include <Scene/TransformationNode.h>

#include <Geometry/AABBShape.h>
#include <Geometry/PlaneShape.h>
#include <Geometry/CompoundShape.h>
#include <Geometry/Ray.h>

#include <typeinfo>

#include <Logging/Logger.h>

namespace OpenEngine {
    namespace Physics {

using namespace Math;
using namespace Geometry;
using namespace std;

// some conversion helper functions
btVector3 BulletPhysics::ToBtVec(Vector<3,float> v) {
        return btVector3(v.Get(0), v.Get(1), v.Get(2));
}

btQuaternion BulletPhysics::ToBtQuat(Quaternion<float> q) {
    Vector<3,float> im = q.GetImaginary();
    return btQuaternion(im.Get(0), im.Get(1), im.Get(2), q.GetReal());
} 

Vector<3,float> BulletPhysics::ToOEVec(btVector3 v) {
    return Vector<3,float>(v.x(), v.y(), v.z());
}

Quaternion<float> BulletPhysics::ToOEQuat(btQuaternion q) {
    return Quaternion<float>(q.w(), q.x(), q.y(), q.z());
} 

btTransform BulletPhysics::ToBtTrans(TransformationNode* tn) {
    return btTransform(ToBtQuat(tn->GetRotation()), ToBtVec(tn->GetPosition()));
} 

btCollisionShape* BulletPhysics::ToBtShape(ICollisionShape* shape, btTransform* trans) {
    trans->setIdentity();
    if (typeid(*shape) == typeid(AABBShape)) {
        AABBShape* s = dynamic_cast<AABBShape*>(shape);
        Vector<3,float> corner = (s->GetMax() - s->GetMin()) * 0.5;
        trans->setOrigin(ToBtVec(s->GetMax() - corner));
        return new btBoxShape(ToBtVec(corner));
    }
    else if (typeid(*shape) == typeid(PlaneShape)) {
        PlaneShape* s = dynamic_cast<PlaneShape*>(shape);
        return new btStaticPlaneShape(ToBtVec(s->GetNormal()), s->GetDistance());
    }
    else if (typeid(*shape) == typeid(CompoundShape)) {
        CompoundShape* s = dynamic_cast<CompoundShape*>(shape);
        btCompoundShape* btShape = new btCompoundShape();
        vector<ICollisionShape*> shapes = s->GetShapes();
        vector<ICollisionShape*>::iterator itr = shapes.begin();
        btTransform t;
        for (; itr != shapes.end(); ++itr) {
            btCollisionShape* cs = ToBtShape(*itr, &t);
            btShape->addChildShape(t, cs);
        }
        return btShape;
    }
    else throw Exception("BulletPhysics: Unsupported bounding shape.");
} 

BulletPhysics::BulletPhysics(AABBShape worldBox)
    : world(NULL)
    , broadphase(NULL)
    , dispatcher(NULL)
    , solver(NULL)
    , conf(NULL)
{
 
    conf = new btDefaultCollisionConfiguration();

    broadphase    = new bt32BitAxisSweep3(ToBtVec(worldBox.GetMin()), ToBtVec(worldBox.GetMax()));
    dispatcher    = new btCollisionDispatcher(conf);
    solver        = new btSequentialImpulseConstraintSolver();

    world = new btDiscreteDynamicsWorld(dispatcher,
                                        broadphase,
                                        solver,
                                        conf);
   
}

BulletPhysics::~BulletPhysics() {
    delete world;
    delete solver;
    delete dispatcher;
    delete broadphase;
    delete conf;
}

void BulletPhysics::Handle(Core::InitializeEventArg arg) {
}

void BulletPhysics::Handle(Core::ProcessEventArg arg) {
    // take a simulation step
    const float dt = arg.approx * 1e-6;
    world->stepSimulation(dt, 10);

    // vector<BulletRigidBody*>::iterator itr = rbs.begin();
    // for (; itr != rbs.end(); ++itr) {
    //     BulletRigidBody* rb = *itr;
    //     rb->body->clearForces();
    // }
}

void BulletPhysics::Handle(Core::DeinitializeEventArg arg) {
}

IRigidBody* BulletPhysics::CreateRigidBody(ICollisionShape* shape, TransformationNode* node) {
    const float mass = 1.0;
    if (!node) {
        node = new TransformationNode();
    }
    btMotionState* motionState = new OEMotionState(node);
    btTransform trans;

    if (typeid(*shape) == typeid(AABBShape)) {
        CompoundShape* cs = new CompoundShape();
        cs->AddShape(shape);
        shape = cs;
    }

    btCollisionShape* btShape = ToBtShape(shape, &trans);
    btVector3 localInertia;
    btShape->calculateLocalInertia(mass, localInertia);
    btRigidBody::btRigidBodyConstructionInfo cInfo(mass, motionState, btShape, localInertia);
    btRigidBody* body = new btRigidBody(cInfo);
    // body->setCenterOfMassTransform(trans);
    world->addRigidBody(body);

    BulletRigidBody* rb = new BulletRigidBody(body, node);
    rbs.push_back(rb);
    
    return rb;
}

void BulletPhysics::AddShape(ICollisionShape* shape) {

    btTransform trans;
    btCollisionShape* btShape = ToBtShape(shape, &trans);

    // world->addCollisionObject(btShape);
}

IRigidBody* BulletPhysics::CastRay(Vector<3,float> begin, Vector<3,float> end, Vector<3,float>* hitPoint) {
    // logger.info << "begin: " << begin << logger.end;
    // logger.info << "end: " << end << logger.end;
    btCollisionWorld::ClosestRayResultCallback cb(ToBtVec(begin), ToBtVec(end));
    world->rayTest(ToBtVec(begin), ToBtVec(end), cb);
    // logger.info << "has hit: " << cb.hasHit() << logger.end;
    vector<BulletRigidBody*>::iterator itr = rbs.begin();
    for (; itr != rbs.end(); ++itr) {
        BulletRigidBody* rb = *itr;
        if (cb.m_collisionObject == rb->body) {
            // logger.info << "hit fraction: " << cb.m_closestHitFraction << logger.end;
            // logger.info << "hit point world: " << BulletPhysics::ToOEVec(cb.m_hitPointWorld) << logger.end;
            if (hitPoint) *hitPoint = ToOEVec(cb.m_hitPointWorld) - rb->GetTransformationNode()->GetPosition();
            return rb;
        }
    }
    return NULL;
}


void BulletPhysics::SetGravity(const Vector<3,float> g) {
    world->setGravity(ToBtVec(g));
}

Vector<3,float> BulletPhysics::GetGravity() const {
    return ToOEVec(world->getGravity());
}

BulletPhysics::OEMotionState::OEMotionState(TransformationNode* node)
    : node(node)
{

}
 
BulletPhysics::OEMotionState::~OEMotionState() {

}

void BulletPhysics::OEMotionState::getWorldTransform (btTransform &worldTrans) const {
    // maybe read accumulated transformations?
    worldTrans.setRotation(ToBtQuat(node->GetRotation()));
    worldTrans.setOrigin(ToBtVec(node->GetPosition()));
}

void BulletPhysics::OEMotionState::setWorldTransform (const btTransform &worldTrans) {
    node->SetPosition(ToOEVec(worldTrans.getOrigin()));
    node->SetRotation(ToOEQuat(worldTrans.getRotation()));
}

BulletPhysics::BulletRigidBody::BulletRigidBody(btRigidBody* body, TransformationNode* node)
  : body(body) 
  , node(node)
{
    
}

BulletPhysics::BulletRigidBody::~BulletRigidBody() {
    
}

TransformationNode* BulletPhysics::BulletRigidBody::GetTransformationNode() {
    return node;
}

void BulletPhysics::BulletRigidBody::ApplyForce(const Vector<3,float> f) {
    body->applyForce(ToBtVec(f), btVector3(0.0, 0.0, 0.0));
    body->setActivationState(1);
}

void BulletPhysics::BulletRigidBody::ApplyForce(const Vector<3,float> f, const Vector<3,float> relPos) {
    body->applyForce(ToBtVec(f), ToBtVec(relPos));
    body->setActivationState(1);
}

void BulletPhysics::BulletRigidBody::ApplyTorque(const Vector<3,float> v) {
    body->applyTorque(ToBtVec(v));
    body->setActivationState(1);
}

void BulletPhysics::BulletRigidBody::SetLinearDamping(float damp) {
    body->setDamping(damp, body->getAngularDamping());    
}

void BulletPhysics::BulletRigidBody::SetAngularDamping(float damp) {
    body->setDamping(body->getLinearDamping(), damp);    
}

void BulletPhysics::BulletRigidBody::SetMass(float mass) {
    btVector3 localInertia;
    body->getCollisionShape()->calculateLocalInertia(mass, localInertia);
    body->setMassProps(mass, localInertia);
}

}
}
