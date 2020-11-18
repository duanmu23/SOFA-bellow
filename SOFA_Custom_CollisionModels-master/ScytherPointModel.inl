
#ifndef SOFA_SCYTHER_POINT_COLLISION_MODEL_INL
#define SOFA_SCYTHER_POINT_COLLISION_MODEL_INL

#include <algorithm>
#include <iostream>
#include <sofa/defaulttype/Mat.h>
#include <sofa/defaulttype/Vec.h>
#include <sofa/helper/proximity.h>
#include <sofa/helper/system/config.h>

#include <SofaMeshCollision/PointLocalMinDistanceFilter.h>
#include <sofa/core/ObjectFactory.h>
#include <sofa/core/visual/VisualParams.h>
#include <vector>

#include <sofa/core/topology/BaseMeshTopology.h>

#include <sofa/simulation/Simulation.h>

#include "ScytherCubeModel.h"
#include "ScytherPointModel.h"

namespace SofaInterface {

template <class DataTypes>
ScytherPointCollisionModel<DataTypes>::ScytherPointCollisionModel()
    : bothSide(initData(
          &bothSide, false, "bothSide",
          "activate collision on both side of the point model (when surface normals are defined on these points)"))
    , mstate(NULL)
    , computeNormals(initData(
          &computeNormals, false, "computeNormals",
          "activate computation of normal vectors (required for some collision detection algorithms)"))
    , PointActiverPath(initData(
          &PointActiverPath, "PointActiverPath",
          "path of a component PointActiver that activate or deactivate collision point during execution"))
    //    , m_lmdFilter( NULL )
    , m_displayFreePosition(initData(
          &m_displayFreePosition, false, "displayFreePosition",
          "Display Collision Model Points free position(in green)"))
{
    enum_type = POINT_TYPE;
}

template <class DataTypes>
void ScytherPointCollisionModel<DataTypes>::resize(int size)
{
    this->sofa::core::CollisionModel::resize(size);
}

template <class DataTypes>
void ScytherPointCollisionModel<DataTypes>::init()
{
    this->CollisionModel::init();
    mstate = dynamic_cast<sofa::core::behavior::MechanicalState<DataTypes>*>(getContext()->getMechanicalState());

    if (mstate == NULL)
    {
        serr << "ERROR: ScytherPointCollisionModel requires a Vec3 Mechanical Model" << sendl;
        return;
    }

    //    sofa::simulation::Node* node = dynamic_cast< sofa::simulation::Node* >(this->getContext());
    //    if (node != 0)
    //    {
    //        m_lmdFilter = node->getNodeObject< PointLocalMinDistanceFilter >();
    //    }

    const int npoints = mstate->getSize();
    resize(npoints);
    if (computeNormals.getValue())
        updateNormals();

    const std::string path = PointActiverPath.getValue();

    if (path.size() == 0)
    {

        myActiver = PointActiver::getDefaultActiver();
        sout << "path = " << path << " no Point Activer found for ScytherPointCollisionModel " << this->getName()
             << sendl;
    }
    else
    {

        sofa::core::objectmodel::BaseObject* activer = NULL;
        this->getContext()->get(activer, path);

        if (activer != NULL)
            sout << " Activer named" << activer->getName() << " found" << sendl;
        else
            serr << "wrong path for PointActiver" << sendl;

        myActiver = dynamic_cast<PointActiver*>(activer);

        if (myActiver == NULL)
        {
            myActiver = PointActiver::getDefaultActiver();

            serr << "no dynamic cast possible for Point Activer for ScytherPointCollisionModel " << this->getName()
                 << sendl;
        }
        else
        {
            sout << "PointActiver named" << activer->getName() << " found !! for ScytherPointCollisionModel "
                 << this->getName() << sendl;
        }
    }
}

template <class DataTypes>
void ScytherPointCollisionModel<DataTypes>::draw(const sofa::core::visual::VisualParams*, int index)
{
    SOFA_UNUSED(index);
    // TODO(fred roy 2018-06-21)...please implement.
}

template <class DataTypes>
void ScytherPointCollisionModel<DataTypes>::draw(const sofa::core::visual::VisualParams* vparams)
{
    if (vparams->displayFlags().getShowCollisionModels())
    {
        if (vparams->displayFlags().getShowWireFrame())
            vparams->drawTool()->setPolygonMode(0, true);

        // Check topological modifications
        const int npoints = mstate->getSize();
        if (npoints != size)
            return;

        std::vector<sofa::defaulttype::Vector3> pointsP;
        std::vector<sofa::defaulttype::Vector3> pointsL;
        for (int i = 0; i < size; i++)
        {
            TScytherPoint<DataTypes> p(this, i);
            if (p.activated())
            {
                pointsP.push_back(p.p());
                if ((unsigned)i < normals.size())
                {
                    pointsL.push_back(p.p());
                    pointsL.push_back(p.p() + normals[i] * 0.1f);
                }
            }
        }

        vparams->drawTool()->drawPoints(pointsP, 3, sofa::defaulttype::Vec<4, float>(getColor4f()));
        vparams->drawTool()->drawLines(pointsL, 1, sofa::defaulttype::Vec<4, float>(getColor4f()));

        if (m_displayFreePosition.getValue())
        {
            std::vector<sofa::defaulttype::Vector3> pointsPFree;

            for (int i = 0; i < size; i++)
            {
                TScytherPoint<DataTypes> p(this, i);
                if (p.activated())
                {
                    pointsPFree.push_back(p.pFree());
                }
            }

            vparams->drawTool()->drawPoints(pointsPFree, 3, sofa::defaulttype::Vec<4, float>(0.0f, 1.0f, 0.2f, 1.0f));
        }

        if (vparams->displayFlags().getShowWireFrame())
            vparams->drawTool()->setPolygonMode(0, false);
    }

    if (getPrevious() != NULL && vparams->displayFlags().getShowBoundingCollisionModels())
        getPrevious()->draw(vparams);
}

template <class DataTypes>
bool ScytherPointCollisionModel<DataTypes>::canCollideWithElement(int index, CollisionModel* model2, int index2)
{

    if (!this->bSelfCollision.getValue())
        return true; // we need to perform this verification process only for the selfcollision case.
    if (this->getContext() != model2->getContext())
        return true;

    bool debug = false;

    if (model2 == this)
    {

        if (index <=
            index2) // to avoid to have two times the same auto-collision we only consider the case when index > index2
            return false;

        sofa::core::topology::BaseMeshTopology* topology = this->getMeshTopology();

        // in the neighborhood, if we find a point in common, we cancel the collision
        const sofa::helper::vector<unsigned int>& verticesAroundVertex1 = topology->getVerticesAroundVertex(index);
        const sofa::helper::vector<unsigned int>& verticesAroundVertex2 = topology->getVerticesAroundVertex(index2);

        for (unsigned int i1 = 0; i1 < verticesAroundVertex1.size(); i1++)
        {

            unsigned int v1 = verticesAroundVertex1[i1];

            for (unsigned int i2 = 0; i2 < verticesAroundVertex2.size(); i2++)
            {

                if (debug)
                    std::cout << "v1 = " << v1 << "  verticesAroundVertex2[i2]" << verticesAroundVertex2[i2]
                              << std::endl;
                if (v1 == verticesAroundVertex2[i2] || v1 == (unsigned int)index2 ||
                    index == (int)verticesAroundVertex2[i2])
                {
                    if (debug)
                        std::cout << " return false" << std::endl;
                    return false;
                }
            }
        }
        if (debug)
            std::cout << " return true" << std::endl;
        return true;
    }
    else
        return model2->canCollideWithElement(index2, this, index);
}

template <class DataTypes>
void ScytherPointCollisionModel<DataTypes>::computeBoundingTree(int maxDepth)
{

    ScytherCubeModel* cubeModel = createPrevious<ScytherCubeModel>();

    //    std::cout << cubeModel->getClassName() << "  " << cubeModel->getPathName() << std::endl;
    //    std::cout << "    " << mstate->getSize() << "  " << size << "  " << std::endl;
    //    std::cout << "    " << cubeModel->getNumberCells() << "  " << cubeModel->getSize() << std::endl;

    const int npoints = mstate->getSize();
    bool      updated = false;
    if (npoints != size)
    {
        resize(npoints);
        updated = true;
    }
    if (updated)
        cubeModel->resize(0);
    if (!isMoving() && !cubeModel->empty() && !updated)
        return; // No need to recompute BBox if immobile

    if (computeNormals.getValue())
        updateNormals();

    cubeModel->resize(size);
    if (!empty())
    {
        // VecCoord& x =mstate->read(core::ConstVecCoordId::position())->getValue();
        const SReal distance = this->proximity.getValue();
        for (int i = 0; i < size; i++)
        {
            TScytherPoint<DataTypes>          p(this, i);
            const sofa::defaulttype::Vector3& pt = p.p();
            cubeModel->setParentOf(
                i, pt - sofa::defaulttype::Vector3(distance, distance, distance),
                pt + sofa::defaulttype::Vector3(distance, distance, distance));
        }
        cubeModel->computeBoundingTree(maxDepth);
    }

    //    if (m_lmdFilter != 0)
    //    {
    //        m_lmdFilter->invalidate();
    //    }
}

// template <class DataTypes>
// void ScytherPointCollisionModel<DataTypes>::computeContinuousBoundingTree(double dt, int maxDepth)
//{
//}

template <class DataTypes>
void ScytherPointCollisionModel<DataTypes>::updateNormals()
{
    const VecCoord& x = this->mstate->read(sofa::core::ConstVecCoordId::position())->getValue();
    int             n = x.size();
    normals.resize(n);
    for (int i = 0; i < n; ++i)
    {
        normals[i].clear();
    }
    sofa::core::topology::BaseMeshTopology* mesh = getContext()->getMeshTopology();
    if (mesh->getNbTetrahedra() + mesh->getNbHexahedra() > 0)
    {
        if (mesh->getNbTetrahedra() > 0)
        {
            const sofa::core::topology::BaseMeshTopology::SeqTetrahedra& elems = mesh->getTetrahedra();
            for (unsigned int i = 0; i < elems.size(); ++i)
            {
                const sofa::core::topology::BaseMeshTopology::Tetra& e  = elems[i];
                const Coord&                                         p1 = x[e[0]];
                const Coord&                                         p2 = x[e[1]];
                const Coord&                                         p3 = x[e[2]];
                const Coord&                                         p4 = x[e[3]];
                Coord&                                               n1 = normals[e[0]];
                Coord&                                               n2 = normals[e[1]];
                Coord&                                               n3 = normals[e[2]];
                Coord&                                               n4 = normals[e[3]];
                Coord                                                n;
                n = cross(p3 - p1, p2 - p1);
                n.normalize();
                n1 += n;
                n2 += n;
                n3 += n;
                n = cross(p4 - p1, p3 - p1);
                n.normalize();
                n1 += n;
                n3 += n;
                n4 += n;
                n = cross(p2 - p1, p4 - p1);
                n.normalize();
                n1 += n;
                n4 += n;
                n2 += n;
                n = cross(p3 - p2, p4 - p2);
                n.normalize();
                n2 += n;
                n4 += n;
                n3 += n;
            }
        }
        /// @todo Hexahedra
    }
    else if (mesh->getNbTriangles() + mesh->getNbQuads() > 0)
    {
        if (mesh->getNbTriangles() > 0)
        {
            const sofa::core::topology::BaseMeshTopology::SeqTriangles& elems = mesh->getTriangles();
            for (unsigned int i = 0; i < elems.size(); ++i)
            {
                const sofa::core::topology::BaseMeshTopology::Triangle& e  = elems[i];
                const Coord&                                            p1 = x[e[0]];
                const Coord&                                            p2 = x[e[1]];
                const Coord&                                            p3 = x[e[2]];
                Coord&                                                  n1 = normals[e[0]];
                Coord&                                                  n2 = normals[e[1]];
                Coord&                                                  n3 = normals[e[2]];
                Coord                                                   n;
                n = cross(p2 - p1, p3 - p1);
                n.normalize();
                n1 += n;
                n2 += n;
                n3 += n;
            }
        }
        if (mesh->getNbQuads() > 0)
        {
            const sofa::core::topology::BaseMeshTopology::SeqQuads& elems = mesh->getQuads();
            for (unsigned int i = 0; i < elems.size(); ++i)
            {
                const sofa::core::topology::BaseMeshTopology::Quad& e  = elems[i];
                const Coord&                                        p1 = x[e[0]];
                const Coord&                                        p2 = x[e[1]];
                const Coord&                                        p3 = x[e[2]];
                const Coord&                                        p4 = x[e[3]];
                Coord&                                              n1 = normals[e[0]];
                Coord&                                              n2 = normals[e[1]];
                Coord&                                              n3 = normals[e[2]];
                Coord&                                              n4 = normals[e[3]];
                Coord                                               n;
                n = cross(p3 - p1, p4 - p2);
                n.normalize();
                n1 += n;
                n2 += n;
                n3 += n;
                n4 += n;
            }
        }
    }
    for (int i = 0; i < n; ++i)
    {
        SReal l = normals[i].norm();
        if (l > 1.0e-3)
            normals[i] *= 1 / l;
        else
            normals[i].clear();
    }
}

template <class DataTypes>
bool TScytherPoint<DataTypes>::testLMD(const sofa::defaulttype::Vector3& PQ, double& coneFactor, double& coneExtension)
{

    sofa::defaulttype::Vector3 pt = p();

    sofa::core::topology::BaseMeshTopology* mesh = this->model->getMeshTopology();
    const typename DataTypes::VecCoord&     x =
        (*this->model->mstate->read(sofa::core::ConstVecCoordId::position())->getValue());

    const sofa::helper::vector<unsigned int>& trianglesAroundVertex = mesh->getTrianglesAroundVertex(this->index);
    const sofa::helper::vector<unsigned int>& edgesAroundVertex     = mesh->getEdgesAroundVertex(this->index);

    sofa::defaulttype::Vector3 nMean;

    for (unsigned int i = 0; i < trianglesAroundVertex.size(); i++)
    {
        unsigned int                                      t    = trianglesAroundVertex[i];
        const sofa::helper::fixed_array<unsigned int, 3>& ptr  = mesh->getTriangle(t);
        sofa::defaulttype::Vector3                        nCur = (x[ptr[1]] - x[ptr[0]]).cross(x[ptr[2]] - x[ptr[0]]);
        nCur.normalize();
        nMean += nCur;
    }

    if (trianglesAroundVertex.size() == 0)
    {
        for (unsigned int i = 0; i < edgesAroundVertex.size(); i++)
        {
            unsigned int                                      e   = edgesAroundVertex[i];
            const sofa::helper::fixed_array<unsigned int, 2>& ped = mesh->getEdge(e);
            sofa::defaulttype::Vector3                        l   = (pt - x[ped[0]]) + (pt - x[ped[1]]);
            l.normalize();
            nMean += l;
        }
    }

    if (nMean.norm() > 0.0000000001)
        nMean.normalize();

    for (unsigned int i = 0; i < edgesAroundVertex.size(); i++)
    {
        unsigned int                                      e   = edgesAroundVertex[i];
        const sofa::helper::fixed_array<unsigned int, 2>& ped = mesh->getEdge(e);
        sofa::defaulttype::Vector3                        l   = (pt - x[ped[0]]) + (pt - x[ped[1]]);
        l.normalize();
        double computedAngleCone = dot(nMean, l) * coneFactor;
        if (computedAngleCone < 0)
            computedAngleCone = 0.0;
        computedAngleCone += coneExtension;
        if (dot(l, PQ) < -computedAngleCone * PQ.norm())
            return false;
    }
    return true;
}

// template<class DataTypes>
// PointLocalMinDistanceFilter *ScytherPointCollisionModel<DataTypes>::getFilter() const
//{
//    return m_lmdFilter;
//}

// template<class DataTypes>
// void ScytherPointCollisionModel<DataTypes>::setFilter(PointLocalMinDistanceFilter *lmdFilter)
//{
//    m_lmdFilter = lmdFilter;
//}

template <class DataTypes>
void ScytherPointCollisionModel<DataTypes>::computeBBox(const sofa::core::ExecParams* params, bool onlyVisible)
{
    if (!onlyVisible)
        return;

    const int npoints = mstate->getSize();
    if (npoints != size)
        return;

    static const Real max_real   = std::numeric_limits<Real>::max();
    static const Real min_real   = std::numeric_limits<Real>::lowest();
    Real              maxBBox[3] = {min_real, min_real, min_real};
    Real              minBBox[3] = {max_real, max_real, max_real};

    for (int i = 0; i < size; i++)
    {
        Element                           e(this, i);
        const sofa::defaulttype::Vector3& p = e.p();

        for (int c = 0; c < 3; c++)
        {
            if (p[c] > maxBBox[c])
                maxBBox[c] = (Real)p[c];
            else if (p[c] < minBBox[c])
                minBBox[c] = (Real)p[c];
        }
    }

    this->f_bbox.setValue(params, sofa::defaulttype::TBoundingBox<Real>(minBBox, maxBBox));
}

} // namespace SofaInterface

#endif
