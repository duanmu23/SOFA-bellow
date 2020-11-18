
#ifndef SOFA_SCYTHER_INTERSECTION_H
#define SOFA_SCYTHER_INTERSECTION_H
#include "SofaGeneral/config.h"

#include <SofaBaseCollision/BaseProximityIntersection.h>

#include <SofaBaseCollision/MinProximityIntersection.h>
#include <SofaMeshCollision/MeshIntTool.h>
#include <sofa/helper/FnDispatcher.h>
//#include <SofaBaseCollision/CapsuleModel.h>
//#include <SofaBaseCollision/SphereModel.h>
//#include <SofaMeshCollision/LineModel.h>
//#include <SofaMeshCollision/TriangleModel.h>
//#include <SofaMeshCollision/PointModel.h>
//#include <SofaBaseCollision/CubeModel.h>
#include <SofaBaseCollision/IntrUtility3.h>

#include <SofaBaseCollision/CubeModel.h>
#include <SofaMeshCollision/LineModel.h>
#include <SofaMeshCollision/PointModel.h>
#include <SofaMeshCollision/TriangleModel.h>
#include <SofaUserInteraction/RayModel.h>

#include "ScytherCubeModel.h"
#include "ScytherPointModel.h"
#include "ScytherTriangleModel.h"
#include "SofaScytherObject.h"

namespace SofaInterface {

class SOFA_BASE_COLLISION_API ScytherIntersection : public sofa::component::collision::BaseProximityIntersection
{
    //    typedef sofa::component::collision::MinProximityIntersection::OutputVector
    //        OutputVector; // should take from elsewhere ???

    template <class T>
    using Data = sofa::core::objectmodel::Data<T>;

    using CubeModel     = sofa::component::collision::CubeModel;
    using PointModel    = sofa::component::collision::PointModel;
    using LineModel     = sofa::component::collision::LineModel;
    using TriangleModel = sofa::component::collision::TriangleModel;
    using RayModel      = sofa::component::collision::RayModel;

    using Cube     = sofa::component::collision::Cube;
    using Point    = sofa::component::collision::Point;
    using Line     = sofa::component::collision::Line;
    using Triangle = sofa::component::collision::Triangle;
    using Ray      = sofa::component::collision::Ray;

public:
    SOFA_CLASS(ScytherIntersection, BaseProximityIntersection);
    Data<bool> usePointPoint;     ///< activate Point-Point intersection tests
    Data<bool> useSurfaceNormals; ///< Compute the norms of the Detection Outputs by considering the normals of the
                                  ///< surfaces involved.

protected:
    ScytherIntersection();

public:
    typedef sofa::core::collision::IntersectorFactory<ScytherIntersection> IntersectorFactory;

    void init() override;

    bool getUseSurfaceNormals();

    void draw(const sofa::core::visual::VisualParams* vparams) override;


    bool testIntersection(ScytherCube&, ScytherCube&);
    bool testIntersection(ScytherPoint&, ScytherPoint&);
    bool testIntersection(ScytherTriangle&, ScytherPoint&);

    bool testIntersection(ScytherCube&, Cube&) { return false; }


    int computeIntersection(ScytherCube&, ScytherCube&, OutputVector*);
    int computeIntersection(ScytherPoint&, ScytherPoint&, OutputVector*);
    int computeIntersection(ScytherTriangle&, ScytherPoint&, OutputVector*);

    int computeIntersection(ScytherCube&, Cube&, OutputVector*) { return 0; }

    /// Actions to accomplish when the broadPhase is started. By default do nothing.
    void beginBroadPhase() override;
    void endBroadPhase() override;

private:
    SReal mainAlarmDistance;
    SReal mainContactDistance;

    int n;

    //    ScytherIntersection(sofa::component::collision::MinProximityIntersection* object, bool addSelf=true);

    // protected:
    //    sofa::component::collision::MinProximityIntersection* intersection;
};

} // namespace SofaInterface



#endif
