
#define SCYTHER_COLLISION_TRIANGLE_MODEL_CPP
#include <sofa/core/ObjectFactory.h>

#include "ScytherTriangleModel.inl"

namespace SofaInterface {

int ScytherTriangleCollisionModelClass =
    sofa::core::RegisterObject("collision model using a triangular mesh, as described in BaseMeshTopology, for Scyther")
        .add<ScytherTriangleCollisionModel<sofa::defaulttype::Vec3Types>>()
        .addAlias("ScytherTriangleModel");
;

template class SOFA_MESH_COLLISION_API ScytherTriangleCollisionModel<sofa::defaulttype::Vec3Types>;

} // namespace SofaInterface
