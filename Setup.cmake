INCLUDE(${OE_CURRENT_EXTENSION_DIR}/FindBullet.cmake)

IF (BULLET_FOUND) 
  INCLUDE_DIRECTORIES(${BULLET_INCLUDE_DIR})
ELSE (BULLET_FOUND)
  MESSAGE ("WARNING: Could not find Bullet physics libraries  - depending targets will be disabled.")
ENDIF (BULLET_FOUND)

