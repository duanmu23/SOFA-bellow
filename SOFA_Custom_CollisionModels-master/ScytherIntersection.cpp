
#include <sofa/helper/system/config.h>

// from MinProximityIntersection
#include <SofaBaseCollision/BaseIntTool.h>
#include <algorithm>
#include <iostream>
#include <sofa/core/ObjectFactory.h>
#include <sofa/core/collision/Intersection.inl>
#include <sofa/core/visual/VisualParams.h>
#include <sofa/defaulttype/Mat.h>
#include <sofa/defaulttype/Vec.h>
#include <sofa/helper/proximity.h>

// extra from MeshMinProximityIntersection
#include <SofaBaseCollision/DiscreteIntersection.h>
#include <sofa/core/collision/IntersectorFactory.h>
#include <sofa/helper/FnDispatcher.inl>

#include <SofaMeshCollision/MeshNewProximityIntersection.inl>

#include "ScytherIntersection.h"

namespace sofa {
namespace core {
namespace collision {
template class SOFA_BASE_COLLISION_API IntersectorFactory<SofaInterface::ScytherIntersection>;
}
} // namespace core
} // namespace sofa

namespace SofaInterface {

using namespace sofa::defaulttype;
using namespace sofa::core::collision;
using namespace sofa::helper;

// from MinProximityIntersection

int ScytherIntersectionClass =
    sofa::core::RegisterObject(
        "A set of methods to compute if two primitives are close enough to consider they collide")
        .add<ScytherIntersection>();

ScytherIntersection::ScytherIntersection()
    : BaseProximityIntersection()
    //, useSphereTriangle(initData(&useSphereTriangle, true, "useSphereTriangle","activate Sphere-Triangle intersection
    // tests"))
    , usePointPoint(initData(&usePointPoint, true, "usePointPoint", "activate Point-Point intersection tests"))
//, useSurfaceNormals(initData(&useSurfaceNormals, false, "useSurfaceNormals", "Compute the norms of the Detection
// Outputs by considering the normals of the surfaces involved.")) , useLinePoint(initData(&useLinePoint, true,
//"useLinePoint", "activate Line-Point intersection tests")) , useLineLine(initData(&useLineLine, true, "useLineLine",
//"activate Line-Line  intersection tests"))
{
}

void ScytherIntersection::init()
{
    intersectors.add<ScytherCubeModel, ScytherCubeModel, ScytherIntersection>(this);

    intersectors.add<ScytherPointModel, ScytherPointModel, ScytherIntersection>(this);
    intersectors.add<ScytherTriangleModel, ScytherPointModel, ScytherIntersection>(this);

    intersectors.add<ScytherCubeModel, CubeModel, ScytherIntersection>(this);

    IntersectorFactory::getInstance()->addIntersectors(this);

    BaseProximityIntersection::init();
}

bool ScytherIntersection::getUseSurfaceNormals()
{
    return useSurfaceNormals.getValue();
}

void ScytherIntersection::draw(const sofa::core::visual::VisualParams* vparams)
{
    if (!vparams->displayFlags().getShowCollisionModels())
        return;
}



bool ScytherIntersection::testIntersection(ScytherCube& cube1, ScytherCube& cube2)
{
    std::cout << "[ScytherIntersection]     running testIntersection(ScytherCube&, ScytherCube&)" << std::endl;

    const sofa::defaulttype::Vector3& minVect1 = cube1.minVect();
    const sofa::defaulttype::Vector3& minVect2 = cube2.minVect();
    const sofa::defaulttype::Vector3& maxVect1 = cube1.maxVect();
    const sofa::defaulttype::Vector3& maxVect2 = cube2.maxVect();

    const double alarmDist = getAlarmDistance() + cube1.getProximity() + cube2.getProximity();

    for (int i = 0; i < 3; i++)
    {
        if (minVect1[i] > maxVect2[i] + alarmDist || minVect2[i] > maxVect1[i] + alarmDist)
        {
            std::cout << "                false" << std::endl;
            return false;
        }
    }
    std::cout << "                true" << std::endl;
    return true;
}

bool ScytherIntersection::testIntersection(ScytherPoint& e1, ScytherPoint& e2)
{
    std::cout << "[ScytherIntersection]     running testIntersection(ScytherPoint&, ScytherPoint&)" << std::endl;

    const SReal alarmDist = getAlarmDistance() + e1.getProximity() + e2.getProximity();

    Vector3 PQ = e2.p() - e1.p();

    if (PQ.norm2() < alarmDist * alarmDist)
        return true;
    else
        return false;
}

bool ScytherIntersection::testIntersection(ScytherTriangle& e2, ScytherPoint& e1)
{
    std::cout << "[ScytherIntersection]     running testIntersection(ScytherTriangle&, ScytherPoint&)" << std::endl;

    const SReal alarmDist = getAlarmDistance() + e1.getProximity() + e2.getProximity();

    const Vector3 AB = e2.p2() - e2.p1();
    const Vector3 AC = e2.p3() - e2.p1();
    const Vector3 AP = e1.p() - e2.p1();
    Matrix2       A;
    Vector2       b;

    // We want to find alpha,beta so that:
    // AQ = AB*alpha+AC*beta
    // PQ.AB = 0 and PQ.AC = 0
    // (AQ-AP).AB = 0 and (AQ-AP).AC = 0
    // AQ.AB = AP.AB and AQ.AC = AP.AC
    //
    // (AB*alpha+AC*beta).AB = AP.AB and
    // (AB*alpha+AC*beta).AC = AP.AC
    //
    // AB.AB*alpha + AC.AB*beta = AP.AB and
    // AB.AC*alpha + AC.AC*beta = AP.AC
    //
    // A . [alpha beta] = b
    A[0][0] = AB * AB;
    A[1][1] = AC * AC;
    A[0][1] = A[1][0] = AB * AC;
    b[0]              = AP * AB;
    b[1]              = AP * AC;
    const SReal det   = determinant(A);

    SReal alpha = 0.5;
    SReal beta  = 0.5;

    alpha = (b[0] * A[1][1] - b[1] * A[0][1]) / det;
    beta  = (b[1] * A[0][0] - b[0] * A[1][0]) / det;
    if (alpha < 0.000001 || beta < 0.000001 || alpha + beta > 0.999999)
        return false;

    const Vector3 PQ = AB * alpha + AC * beta - AP;

    if (PQ.norm2() < alarmDist * alarmDist)
        return true;
    else
        return false;
}

int ScytherIntersection::computeIntersection(ScytherCube& cube1, ScytherCube& cube2, OutputVector* contacts)
{
    std::cout << "[ScytherIntersection]     running computeIntersection(ScytherCube&, ScytherCube&, ...)" << std::endl;

    sofa::core::CollisionModel* cm1 = cube1.getCollisionModel();
    sofa::core::CollisionModel* cm2 = cube2.getCollisionModel();

    sofa::core::CollisionModel* finalcm1 = cm1->getLast();
    sofa::core::CollisionModel* finalcm2 = cm2->getLast();

    int type1 = finalcm1->getEnumType();
    int type2 = finalcm2->getEnumType();

    int nbContacts = 0;

    for (sofa::core::CollisionElementIterator it1 = finalcm1->begin(); it1 != finalcm1->end(); ++it1)
    {
        for (sofa::core::CollisionElementIterator it2 = finalcm2->begin(); it2 != finalcm2->end(); ++it2)
        {
            if (type1 == sofa::core::CollisionModel::POINT_TYPE && type2 == sofa::core::CollisionModel::POINT_TYPE)
            {
                ScytherPoint e1 = ScytherPoint(it1);
                ScytherPoint e2 = ScytherPoint(it2);
                nbContacts += computeIntersection(e1, e2, contacts);
            }
            else if (
                type1 == sofa::core::CollisionModel::POINT_TYPE && type2 == sofa::core::CollisionModel::TRIANGLE_TYPE)
            {
                ScytherPoint    e2 = ScytherPoint(it1);
                ScytherTriangle e1 = ScytherTriangle(it2);
                nbContacts += computeIntersection(e1, e2, contacts);
            }
            else if (
                type1 == sofa::core::CollisionModel::TRIANGLE_TYPE && type2 == sofa::core::CollisionModel::POINT_TYPE)
            {
                ScytherTriangle e1 = ScytherTriangle(it1);
                ScytherPoint    e2 = ScytherPoint(it2);
                nbContacts += computeIntersection(e1, e2, contacts);
            }
            //            else
            //                std::cout << "ERROR: ScytherIntersection::computeIntersection: type of collision model not
            //                handled"
            //                          << std::endl;
        }
    }

    return nbContacts;
}

int ScytherIntersection::computeIntersection(ScytherPoint& e1, ScytherPoint& e2, OutputVector* contacts)
{
    if (n < 10)
    {
        n++;
        std::cout << "[ScytherIntersection]     running computeIntersection(ScytherPoint&, ScytherPoint&, ...)"
                  << std::endl;
    }

    const SReal alarmDist = getAlarmDistance() + e1.getProximity() + e2.getProximity();

    Vector3 P, Q, PQ;
    P  = e1.p();
    Q  = e2.p();
    PQ = Q - P;

    if (PQ.norm2() >= alarmDist * alarmDist)
        return 0;

    contacts->resize(contacts->size() + 1);
    DetectionOutput* detection = &*(contacts->end() - 1);

#ifdef DETECTIONOUTPUT_FREEMOTION
    if (e1.hasFreePosition() && e2.hasFreePosition())
    {
        Vector3 Pfree, Qfree;
        Pfree = e1.pFree();
        Qfree = e2.pFree();

        detection->freePoint[0] = Pfree;
        detection->freePoint[1] = Qfree;
    }
#endif

    const SReal contactDist = getContactDistance() + e1.getProximity() + e2.getProximity();

    detection->elem = std::pair<sofa::core::CollisionElementIterator, sofa::core::CollisionElementIterator>(e1, e2);
    detection->id =
        (e1.getCollisionModel()->getSize() > e2.getCollisionModel()->getSize()) ? e1.getIndex() : e2.getIndex();
    detection->point[0] = P;
    detection->point[1] = Q;
    detection->normal   = PQ;
    detection->value    = detection->normal.norm();

    if (detection->value > 1e-15)
    {
        detection->normal /= detection->value;
    }
    else
    {
        serr << "WARNING: null distance between contact detected" << sendl;
        detection->normal = Vector3(1, 0, 0);
    }
    detection->value -= contactDist;

    if (n < 10)
    {
        std::cout << "                " << detection->point[0] << "    " << detection->point[1] << "    "
                  << detection->value << std::endl;
    }

    return 1;
}

int ScytherIntersection::computeIntersection(ScytherTriangle& e2, ScytherPoint& e1, OutputVector* contacts)
{
    if (n < 10)
    {
        n++;
        std::cout << "[ScytherIntersection]     running computeIntersection(ScytherTriangle, ScytherPoint, ...)"
                  << std::endl;
    }

    const SReal alarmDist = getAlarmDistance() + e1.getProximity() + e2.getProximity();

    const Vector3 AB = e2.p2() - e2.p1();
    const Vector3 AC = e2.p3() - e2.p1();
    const Vector3 AP = e1.p() - e2.p1();
    Matrix2       A;
    Vector2       b;

    A[0][0] = AB * AB;
    A[1][1] = AC * AC;
    A[0][1] = A[1][0] = AB * AC;
    b[0]              = AP * AB;
    b[1]              = AP * AC;

    const SReal det = determinant(A);

    SReal alpha = 0.5;
    SReal beta  = 0.5;

    alpha = (b[0] * A[1][1] - b[1] * A[0][1]) / det;
    beta  = (b[1] * A[0][0] - b[0] * A[1][0]) / det;
    if (alpha < 0.000001 || beta < 0.000001 || alpha + beta > 0.999999)
        return 0;

    Vector3 P, Q, QP; // PQ
    P  = e1.p();
    Q  = e2.p1() + AB * alpha + AC * beta;
    QP = P - Q;

    if (QP.norm2() >= alarmDist * alarmDist)
        return 0;

    // Vector3 PQ = Q-P;

    contacts->resize(contacts->size() + 1);
    DetectionOutput* detection = &*(contacts->end() - 1);

#ifdef DETECTIONOUTPUT_FREEMOTION
    if (e1.hasFreePosition() && e2.hasFreePosition())
    {
        Vector3 Pfree, Qfree, ABfree, ACfree;
        ABfree = e2.p2Free() - e2.p1Free();
        ACfree = e2.p3Free() - e2.p1Free();
        Pfree  = e1.pFree();
        Qfree  = e2.p1Free() + ABfree * alpha + ACfree * beta;

        detection->freePoint[0] = Qfree;
        detection->freePoint[1] = Pfree;
    }
#endif

    const SReal contactDist = getContactDistance() + e1.getProximity() + e2.getProximity();

    detection->elem     = std::pair<sofa::core::CollisionElementIterator, sofa::core::CollisionElementIterator>(e2, e1);
    detection->id       = e1.getIndex();
    detection->point[0] = Q;
    detection->point[1] = P;
    detection->normal   = QP;
    detection->value    = detection->normal.norm();
    if (detection->value > 1e-15)
    {
        detection->normal /= detection->value;
    }
    else
    {
        serr << "WARNING: null distance between contact detected" << sendl;
        detection->normal = Vector3(1, 0, 0);
    }
    detection->value -= contactDist;

    if (getUseSurfaceNormals())
    {
        int normalIndex   = e2.getIndex();
        detection->normal = e2.model->getNormals()[normalIndex];
    }

    return 1;
}

void ScytherIntersection::beginBroadPhase()
{
    n = 0;
    // std::cout << "[ScytherIntersection]     running ScytherIntersection::beginBroadPhase()" << std::endl;
}

void ScytherIntersection::endBroadPhase()
{
    // std::cout << "[ScytherIntersection]     running ScytherIntersection::endBroadPhase()" << std::endl;
}

} // namespace SofaInterface
