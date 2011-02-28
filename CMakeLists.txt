INCLUDE(FindBullet.cmake)

IF (BULLET_FOUND) 
  INCLUDE_DIRECTORIES(${BULLET_INCLUDE_DIR})
ELSE (BULLET_FOUND)
  MESSAGE ("WARNING: Could not find Bullet physics libraries  - depending targets will be disabled.")
ENDIF (BULLET_FOUND)

# Create the extension library
ADD_LIBRARY(Extensions_BulletPhysics
  Physics/IPhysicsEngine.h
  Physics/IRigidBody.h
  Physics/BulletPhysics.h
  Physics/BulletPhysics.cpp
  Physics/RigidBodyBuilder.h
  Physics/RigidBodyBuilder.cpp

  Renderers/BulletDebugDrawer.h
  Renderers/BulletDebugDrawer.cpp
  
  Geometry/ShapeBuilder.h
  Geometry/ShapeBuilder.cpp
  Geometry/CompoundShape.h
  Geometry/CompoundShape.cpp
  Geometry/AABBShape.h
  Geometry/AABBShape.cpp
  Geometry/PlaneShape.h
  Geometry/PlaneShape.cpp

  Utils/ForceHandler.h
  Utils/ForceHandler.cpp
)

TARGET_LINK_LIBRARIES( Extensions_BulletPhysics
    ${BULLET_LIBRARIES}
)
