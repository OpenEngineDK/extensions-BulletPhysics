// Create bounding shapes from meshes
// -------------------------------------------------------------------
// Copyright (C) 2007 OpenEngine.dk (See AUTHORS) 
// 
// This program is free software; It is covered by the GNU General 
// Public License version 2 or any later version. 
// See the GNU General Public License for more details (see LICENSE). 
//--------------------------------------------------------------------

#include <Geometry/ShapeBuilder.h>

// #include <Scene/ISceneNode.h>
// #include <Scene/MeshNode.h>
#include <Scene/TransformationNode.h>
#include <Geometry/Mesh.h>
#include <Geometry/AABBShape.h>
#include <Geometry/GeometrySet.h>
#include <Resources/IDataBlock.h>

namespace OpenEngine {
namespace Geometry {

using namespace Scene;
using namespace Resources;

ShapeBuilder::ShapeBuilder() {

}
    
ShapeBuilder::~ShapeBuilder() {

}
    
// AABBShape ShapeBuilder::CreateAABBShape(ISceneNode* node) {
//     min = max = Vector<3,float>(0.0);
//     node->Accept(*this);
//     return AABBShape(min, max);
// }

AABBShape ShapeBuilder::CreateAABBShape(Mesh* mesh, TransformationNode* node) {
    IDataBlockPtr verts = mesh->GetGeometrySet()->GetVertices();
    Vector<3,float> lmin(0.0);
    Vector<3,float> lmax(0.0);
    
    for (unsigned int i=0; i < verts->GetSize();i++) {
        Vector<3,float> elm;
        verts->GetElement(i, elm);

        if (node) {
            elm = node->GetRotation().RotateVector(elm);
            elm += node->GetPosition();
        }
        if (elm[0] < lmin[0]) lmin[0] = elm[0];
        if (elm[1] < lmin[1]) lmin[1] = elm[1];
        if (elm[2] < lmin[2]) lmin[2] = elm[2];

        if (elm[0] > lmax[0]) lmax[0] = elm[0];
        if (elm[1] > lmax[1]) lmax[1] = elm[1];
        if (elm[2] > lmax[2]) lmax[2] = elm[2];
    }

    return AABBShape(lmin, lmax);
}

// void ShapeBuilder::VisitMeshNode(MeshNode* node) {
//     // simply get the biggest corners in the tree. doesn't really make
//     // sense to include transformation nodes, or does it? ...
//     AABBShape box = CreateAABBShape(node->GetMesh().get());
//     Vector<3,float> elm = box.GetMin();
//     if (elm[0] < min[0]) min[0] = elm[0];
//     if (elm[1] < min[1]) min[1] = elm[1];
//     if (elm[2] < min[2]) min[2] = elm[2];
//     elm = box.GetMax();
//     if (elm[0] > max[0]) max[0] = elm[0];
//     if (elm[1] > max[1]) max[1] = elm[1];
//     if (elm[2] > max[2]) max[2] = elm[2];
// }

} //NS Geometry
} //NS OpenEngine
