#ifndef _OE_BOUNDING_SPHERE_
#define _OE_BOUNDING_SPHERE_

#include <Geometry/Sphere.h>
#include <Geometry/Geometry.h>

namespace OpenEngine {
namespace Geometry {

class BoundingSphere : public GeometryBase
{
    private:
        Sphere* sphere;
    public:
        BoundingSphere(Vector<3,float> center, float diameter)
        {
            sphere = new Sphere(center, diameter);
        }
        virtual ~BoundingSphere()
        {
            delete sphere;
        }

        Sphere getSphere()
        {
            return *sphere;
        }
};

} // NS Geometry
} // NS OpenEngine

#endif // _OE_BOUNDING_SPHERE_
