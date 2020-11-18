
#define SCYTHER_DETECTION_CPP

//#include <SofaBaseCollision/BruteForceDetection.h>
#include <sofa/core/visual/VisualParams.h>

#include <map>
#include <queue>
#include <sofa/core/ObjectFactory.h>
#include <sofa/helper/AdvancedTimer.h>
#include <sofa/helper/FnDispatcher.h>
#include <stack>

#include <SofaMeshCollision/PointModel.h>

//#include <sofa/core/collision/Contact.h>
//#include <sofa/helper/Factory.inl>

#include "ScytherDetection.h"
#include "ScytherPointModel.h"
#include "ScytherTriangleModel.h"

// namespace sofa {
// namespace component {
// namespace collision {
// template <class Factory, class RealObject>
// using Creator = sofa::helper::Creator<Factory, RealObject>;
// using Contact = sofa::core::collision::Contact;

// Creator<
//    Contact::Factory, BarycentricPenalityContact<
//                          SofaInterface::ScytherPointModel<sofa::defaulttype::Vec3Types>,
//                          SofaInterface::ScytherPointModel<sofa::defaulttype::Vec3Types>>>
//    ScytherPointScytherPointPenalityContactClass("default", true);
// Creator<
//    Contact::Factory, BarycentricPenalityContact<
//                          SofaInterface::ScytherTriangleModel<sofa::defaulttype::Vec3Types>,
//                          SofaInterface::ScytherPointModel<sofa::defaulttype::Vec3Types>>>
//    ScytherTriangleScytherPointPenalityContactClass("default", true);

// template class SOFA_MESH_COLLISION_API BarycentricPenalityContact<
//    SofaInterface::ScytherPointModel<sofa::defaulttype::Vec3Types>,
//    SofaInterface::ScytherPointModel<sofa::defaulttype::Vec3Types>>;
// template class SOFA_MESH_COLLISION_API BarycentricPenalityContact<
//    SofaInterface::ScytherTriangleModel<sofa::defaulttype::Vec3Types>,
//    SofaInterface::ScytherPointModel<sofa::defaulttype::Vec3Types>>;

//} // namespace collision
//} // namespace component
//} // namespace sofa

//

namespace SofaInterface {


int ScytherDetectionClass =
    sofa::core::RegisterObject("Collision detection using extensive pair-wise tests").add<ScytherDetection>();

ScytherDetection::ScytherDetection()
    : box(initData(&box, "box", "if not empty, objects that do not intersect this bounding-box will be ignored"))
{
}

ScytherDetection::~ScytherDetection()
{
}

void ScytherDetection::init()
{
    reinit();
}

void ScytherDetection::reinit()
{
    if (box.getValue()[0][0] >= box.getValue()[1][0])
    {
        boxModel.reset();
    }
    else
    {
        if (!boxModel)
            boxModel = sofa::core::objectmodel::New<ScytherCubeModel>();
        boxModel->resize(1);
        boxModel->setParentOf(0, box.getValue()[0], box.getValue()[1]);
    }
}

void ScytherDetection::addCollisionModel(sofa::core::CollisionModel* cm)
{
    // std::cout << "    ScytherDetection::addCollisionModel" << std::endl;
    std::cout << "[ScytherDetection]    Adding " << cm->getClassName() << " (" << cm->getPathName()
              << ", last=" << cm->getLast()->getClassName() << ")" << std::endl;
    collisionModels.push_back(cm);
}

void ScytherDetection::addCollisionPair(
    const std::pair<sofa::core::CollisionModel*, sofa::core::CollisionModel*>& cmPair)
{
}

void ScytherDetection::beginBroadPhase()
{
    sofa::core::collision::BroadPhaseDetection::beginBroadPhase(); // clear cmPairs
    collisionModels.clear();

    std::cout << std::endl << "[ScytherDetection] BEGINING COLLISION DETECTION" << std::endl;
    //    std::cout << "        output_map: " << m_outputsMap.size() << std::endl;
}

void ScytherDetection::endBroadPhase()
{
    //    std::cout << "    runnign ScytherDetection::endBroadPhase()" << std::endl;
    //    std::cout << "        " << collisionModels.size() << " col models added" << std::endl;
}

void ScytherDetection::beginNarrowPhase()
{
    //    std::cout << "    runnign ScytherDetection::beginNarrowPhase()" << std::endl;

    // clear the potentially colliding pairs from previous step in m_outputsMap
    sofa::core::collision::NarrowPhaseDetection::beginNarrowPhase();

    // Brute force comparaison of all the bounding volumes
    for (sofa::helper::vector<sofa::core::CollisionModel*>::const_iterator it1 = collisionModels.begin();
         it1 < collisionModels.end(); it1++)
    {
        for (sofa::helper::vector<sofa::core::CollisionModel*>::const_iterator it2 = it1 + 1;
             it2 < collisionModels.end(); it2++)
        {
            sofa::core::CollisionModel* cm1 = (*it1);
            sofa::core::CollisionModel* cm2 = (*it2);

            std::cout << "[ScytherDetection]    Doing detection between " << cm1->getPathName()
                      << " (last=" << cm1->getLast()->getClassName()
                      << ")\n                                          and " << cm2->getPathName()
                      << " (last=" << cm2->getLast()->getClassName() << ")" << std::endl;

            bool swapModels = false;

            sofa::core::collision::ElementIntersector* intersector =
                intersectionMethod->findIntersector(cm1, cm2, swapModels);

            if (intersector == NULL)
                std::cout << "[ScytherDetection] ERROR finding intersector " << intersectionMethod->getName() << " for "
                          << cm1->getClassName() << " - " << cm2->getClassName() << std::endl;

            if (swapModels)
            {
                sofa::core::CollisionModel* tmp;
                tmp = cm1;
                cm1 = cm2;
                cm2 = tmp;
            }

            // check if is same object
            const bool self = (cm1->getContext() == cm2->getContext());

            // call testIntersection method of the intersection class
            if (!self && intersector->canIntersect(cm1->begin(), cm2->begin()))
            {
                // Get the contact vector for this pair of collision model, or create a null pointer to
                // it if it doesn't exist
                std::cout << "[ScytherDetection]        output_map: size = " << m_outputsMap.size() << std::endl;
                sofa::core::collision::DetectionOutputVector*& outputs =
                    this->getDetectionOutputs(cm1->getLast(), cm2->getLast());
                std::cout << "[ScytherDetection]        output_map: size = " << m_outputsMap.size() << std::endl;

                intersector->beginIntersect(cm1, cm2, outputs);

                if (outputs == NULL)
                {
                    std::cout << "[ScytherDetection] ERROR: outputs is NULL!" << std::endl;
                    continue;
                }

                std::cout << "[ScytherDetection]        detection output vector size (before intersect) = "
                          << outputs->size() << std::endl;

                // call the computeIntersection method of the intersection class
                intersector->intersect(cm1->begin(), cm2->begin(), outputs);

                std::cout << "[ScytherDetection]        detection output vector size  (after intersect) = "
                          << outputs->size() << std::endl;
            }
        }
    }
}

void ScytherDetection::endNarrowPhase()
{
    sofa::core::collision::NarrowPhaseDetection::endNarrowPhase();

    //    std::cout << "    runnign ScytherDetection::endNarrowPhase()" << std::endl;
    //    std::cout << "        output_map: size = " << m_outputsMap.size() << std::endl;
}

} // namespace SofaInterface
