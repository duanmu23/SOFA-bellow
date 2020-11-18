
#ifndef SCYTHER_DETECTION_H
#define SCYTHER_DETECTION_H
#include "SofaBaseCollision/config.h"

//#include <SofaBaseCollision/CubeModel.h>
#include <sofa/core/CollisionElement.h>
#include <sofa/core/collision/BroadPhaseDetection.h>
#include <sofa/core/collision/NarrowPhaseDetection.h>
#include <sofa/defaulttype/Vec.h>

#include <SofaMeshCollision/BarycentricPenalityContact.h>

#include "ScytherCubeModel.h"
#include "ScytherPointModel.h"
#include "ScytherTriangleModel.h"

namespace SofaInterface {

class SOFA_BASE_COLLISION_API ScytherDetection : public sofa::core::collision::BroadPhaseDetection,
                                                 public sofa::core::collision::NarrowPhaseDetection
{
    template <class T>
    using Data = sofa::core::objectmodel::Data<T>;

public:
    SOFA_CLASS2(
        ScytherDetection, sofa::core::collision::BroadPhaseDetection, sofa::core::collision::NarrowPhaseDetection);

private:
    bool                                              _is_initialized;
    sofa::helper::vector<sofa::core::CollisionModel*> collisionModels;

    Data<sofa::helper::fixed_array<sofa::defaulttype::Vector3, 2>>
        box; ///< if not empty, objects that do not intersect this bounding-box will be ignored

    ScytherCubeModel::SPtr boxModel;

protected:
    ScytherDetection();

    ~ScytherDetection() override;

    // virtual bool keepCollisionBetween(sofa::core::CollisionModel* cm1, sofa::core::CollisionModel* cm2);

public:
    void init() override;
    void reinit() override;

    void addCollisionModel(sofa::core::CollisionModel* cm) override;
    void addCollisionPair(const std::pair<sofa::core::CollisionModel*, sofa::core::CollisionModel*>& cmPair) override;

    void beginBroadPhase() override;
    void endBroadPhase() override;
    void beginNarrowPhase() override;
    void endNarrowPhase() override;

    void draw(const sofa::core::visual::VisualParams* /* vparams */) override {}

    inline bool needsDeepBoundingTree() const override { return false; }
};

// namespace sofa {
// namespace component {
// namespace collision {
//#if !defined(SCYTHER_DETECTION_CPP)
// extern template class SOFA_MESH_COLLISION_API BarycentricPenalityContact<ScytherPointModel, ScytherPointModel>;
// extern template class SOFA_MESH_COLLISION_API BarycentricPenalityContact<ScytherTriangleModel, ScytherPointModel>;
//#endif
// }
// }
// }

} // namespace SofaInterface

//

#endif
