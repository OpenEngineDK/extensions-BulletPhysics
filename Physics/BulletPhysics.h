// Concrete implementation of a physics engine using bullet.
// -------------------------------------------------------------------
// Copyright (C) 2007 OpenEngine.dk (See AUTHORS)
//
// This program is free software; It is covered by the GNU General
// Public License version 2 or any later version.
// See the GNU General Public License for more details (see LICENSE).
//--------------------------------------------------------------------

#ifndef _OE_BULLET_PHYSICS_ENGINE_H_
#define _OE_BULLET_PHYSICS_ENGINE_H_

#include <Physics/IPhysicsEngine.h>
#include <Physics/IRigidBody.h>
#include <Math/Quaternion.h>
#include <Math/Vector.h>

#include <btBulletDynamicsCommon.h>
#include <btBulletCollisionCommon.h>
#include <vector>

// Bullet definitions
// class btRigidBody;
// class btTransform;
// class btCollisionShape;
// class btDynamicsWorld;
// class btBroadphaseInterface;
// class btCollisionDispatcher;
// class btConstraintSolver;
// class btCollisionAlgorithmCreateFunc;
// class btDefaultCollisionConfiguration;

namespace OpenEngine {
    namespace Geometry {
        class ICollisionShape;
        class AABBShape;
        class Ray;
    }
    namespace Scene {
        class TransformationNode;
    }
    namespace Renderers {
        class BulletDebugDrawer;
    }
namespace Physics {


using Scene::TransformationNode;
using Math::Vector;
using Math::Quaternion;
using Geometry::ICollisionShape;
using Geometry::AABBShape;
using Geometry::Ray;
using Renderers::BulletDebugDrawer;

class BulletPhysics : public IPhysicsEngine {
private:
    friend class BulletDebugDrawer;
    // class to read and update transformation nodes.
    class OEMotionState: public btMotionState {
    private:
        TransformationNode* node;
    public:
        OEMotionState(TransformationNode* node);
        virtual ~OEMotionState();
        void getWorldTransform (btTransform &worldTrans) const;
        void setWorldTransform (const btTransform &worldTrans);
  };

    class BulletRigidBody : public IRigidBody {
        friend class BulletPhysics;
    private:
        btRigidBody* body;
        TransformationNode* node;
        BulletRigidBody(btRigidBody* body, TransformationNode* node);
    public:
        virtual ~BulletRigidBody();

        TransformationNode* GetTransformationNode();
        void ApplyForce(const Vector<3,float> f);
        void ApplyForce(const Vector<3,float> f, const Vector<3,float> relPos);
        void ApplyTorque(const Vector<3,float> v);

        void SetLinearDamping(float damp);
        void SetAngularDamping(float damp);
        void SetMass(float mass);
    };

    btDynamicsWorld*       world;
    btBroadphaseInterface* broadphase;  
    btCollisionDispatcher* dispatcher;
    btConstraintSolver*    solver;
    btDefaultCollisionConfiguration* conf;  

    std::vector<BulletRigidBody*> rbs;
public:
    BulletPhysics(AABBShape worldBox);
    virtual ~BulletPhysics();

    void Handle(Core::InitializeEventArg arg);
    void Handle(Core::ProcessEventArg arg);
    void Handle(Core::DeinitializeEventArg arg);
    
    IRigidBody* CreateRigidBody(ICollisionShape* shape, TransformationNode* node = NULL);
    IRigidBody* CastRay(Vector<3,float> begin, Vector<3,float> end, Vector<3,float>* hitPoint = NULL);

    void AddShape(ICollisionShape* shape);

    void SetGravity(const Vector<3,float> g);
    Vector<3,float> GetGravity() const;

    // static conversion methods
    static inline btVector3 ToBtVec(Vector<3,float> v);
    static inline btQuaternion ToBtQuat(Quaternion<float> q);
    static inline Vector<3,float> ToOEVec(btVector3 v);
    static inline Quaternion<float> ToOEQuat(btQuaternion q);
    static inline btTransform ToBtTrans(TransformationNode* tn);
    static inline btCollisionShape* ToBtShape(ICollisionShape* shape, btTransform* trans);
};
    
}
}
#endif //_OE_BULLET_PHYSICS_ENGINE_H_
