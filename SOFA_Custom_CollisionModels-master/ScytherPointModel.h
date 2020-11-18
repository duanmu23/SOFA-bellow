#ifndef SCYTHER_POINT_COLLISION_MODEL_H
#define SCYTHER_POINT_COLLISION_MODEL_H
#include <SofaMeshCollision/config.h>

#include <SofaBaseMechanics/MechanicalObject.h>
//#include <SofaMeshCollision/LocalMinDistanceFilter.h>
#include <sofa/core/CollisionModel.h>
#include <sofa/core/topology/BaseMeshTopology.h>
#include <sofa/defaulttype/VecTypes.h>
#include <vector>

#include <SofaMeshCollision/BarycentricPenalityContact.h>

namespace SofaInterface {

template <class DataTypes>
class ScytherPointCollisionModel;

// class PointLocalMinDistanceFilter;

template <class TDataTypes>
class TScytherPoint : public sofa::core::TCollisionElementIterator<ScytherPointCollisionModel<TDataTypes>>
{
public:
    typedef TDataTypes                            DataTypes;
    typedef typename DataTypes::Coord             Coord;
    typedef typename DataTypes::Deriv             Deriv;
    typedef ScytherPointCollisionModel<DataTypes> ParentModel;

    TScytherPoint(ParentModel* /*model*/, int /*index*/);
    TScytherPoint() {}

    explicit TScytherPoint(const sofa::core::CollisionElementIterator& i);

    const Coord& p() const;
    const Coord& pFree() const;
    const Deriv& v() const;
    Deriv        n() const;

    /// Return true if the element stores a free position vector
    bool hasFreePosition() const;

    bool testLMD(const sofa::defaulttype::Vector3&, double&, double&);

    bool activated(sofa::core::CollisionModel* cm = nullptr) const;
};

class PointActiver
{
public:
    PointActiver() {}
    virtual ~PointActiver() {}
    virtual bool         activePoint(int /*index*/, sofa::core::CollisionModel* /*cm*/ = nullptr) { return true; }
    static PointActiver* getDefaultActiver()
    {
        static PointActiver defaultActiver;
        return &defaultActiver;
    }
};

template <class TDataTypes>
class SOFA_MESH_COLLISION_API ScytherPointCollisionModel : public sofa::core::CollisionModel
{
public:
    SOFA_CLASS(SOFA_TEMPLATE(ScytherPointCollisionModel, TDataTypes), sofa::core::CollisionModel);

    typedef TDataTypes                            DataTypes;
    typedef DataTypes                             InDataTypes;
    typedef ScytherPointCollisionModel<DataTypes> ParentModel;
    typedef typename DataTypes::VecCoord          VecCoord;
    typedef typename DataTypes::VecDeriv          VecDeriv;
    typedef typename DataTypes::Coord             Coord;
    typedef typename DataTypes::Deriv             Deriv;
    typedef TScytherPoint<DataTypes>              Element;
    typedef sofa::helper::vector<unsigned int>    VecIndex;

    friend class TScytherPoint<DataTypes>;

    template <class T>
    using Data = sofa::core::objectmodel::Data<T>;

protected:
    ScytherPointCollisionModel();

public:
    void init() override;

    // -- CollisionModel interface

    void resize(int size) override;

    void computeBoundingTree(int maxDepth = 0) override;

    void computeContinuousBoundingTree(double, int) override {}

    void draw(const sofa::core::visual::VisualParams*, int index) override;
    void draw(const sofa::core::visual::VisualParams* vparams) override;

    bool canCollideWithElement(int index, CollisionModel* model2, int index2) override;

    sofa::core::behavior::MechanicalState<DataTypes>* getMechanicalState() { return mstate; }

    Deriv getNormal(int index) { return (normals.size()) ? normals[index] : Deriv(); }

    // PointLocalMinDistanceFilter *getFilter() const;

    // void setFilter(PointLocalMinDistanceFilter * /*lmdFilter*/);

    const Deriv& velocity(int index) const;

    Data<bool> bothSide; ///< to activate collision on both side of the point model (when surface normals are defined on
                         ///< these points)

    /// Pre-construction check method called by ObjectFactory.
    /// Check that DataTypes matches the MechanicalState.
    template <class T>
    static bool canCreate(
        T*& obj, sofa::core::objectmodel::BaseContext* context, sofa::core::objectmodel::BaseObjectDescription* arg)
    {
        if (dynamic_cast<sofa::core::behavior::MechanicalState<DataTypes>*>(context->getMechanicalState()) == NULL)
            return false;
        return BaseObject::canCreate(obj, context, arg);
    }

    virtual std::string getTemplateName() const override { return templateName(this); }

    static std::string templateName(const ScytherPointCollisionModel<DataTypes>* = NULL) { return DataTypes::Name(); }

    void computeBBox(const sofa::core::ExecParams* params, bool onlyVisible) override;
    void updateNormals();

protected:
    sofa::core::behavior::MechanicalState<DataTypes>* mstate;

    Data<bool>
        computeNormals; ///< activate computation of normal vectors (required for some collision detection algorithms)

    Data<std::string> PointActiverPath; ///< path of a component PointActiver that activate or deactivate collision
                                        ///< point during execution

    VecDeriv normals;

    // PointLocalMinDistanceFilter *m_lmdFilter;
    sofa::component::collision::EmptyFilter m_emptyFilter;

    Data<bool> m_displayFreePosition; ///< Display Collision Model Points free position(in green)

    PointActiver* myActiver;
};

template <class DataTypes>
inline TScytherPoint<DataTypes>::TScytherPoint(ParentModel* model, int index)
    : sofa::core::TCollisionElementIterator<ParentModel>(model, index)
{
}

template <class DataTypes>
inline TScytherPoint<DataTypes>::TScytherPoint(const sofa::core::CollisionElementIterator& i)
    : sofa::core::TCollisionElementIterator<ParentModel>(static_cast<ParentModel*>(i.getCollisionModel()), i.getIndex())
{
}

template <class DataTypes>
inline const typename DataTypes::Coord& TScytherPoint<DataTypes>::p() const
{
    return this->model->mstate->read(sofa::core::ConstVecCoordId::position())->getValue()[this->index];
}

template <class DataTypes>
inline const typename DataTypes::Coord& TScytherPoint<DataTypes>::pFree() const
{
    if (hasFreePosition())
        return this->model->mstate->read(sofa::core::ConstVecCoordId::freePosition())->getValue()[this->index];
    else
        return p();
}

template <class DataTypes>
inline const typename DataTypes::Deriv& TScytherPoint<DataTypes>::v() const
{
    return this->model->mstate->read(sofa::core::ConstVecDerivId::velocity())->getValue()[this->index];
}

template <class DataTypes>
inline const typename DataTypes::Deriv& ScytherPointCollisionModel<DataTypes>::velocity(int index) const
{
    return mstate->read(sofa::core::ConstVecDerivId::velocity())->getValue()[index];
}

template <class DataTypes>
inline typename DataTypes::Deriv TScytherPoint<DataTypes>::n() const
{
    return ((unsigned)this->index < this->model->normals.size()) ? this->model->normals[this->index] : Deriv();
}

template <class DataTypes>
inline bool TScytherPoint<DataTypes>::hasFreePosition() const
{
    return this->model->mstate->read(sofa::core::ConstVecCoordId::freePosition())->isSet();
}

template <class DataTypes>
inline bool TScytherPoint<DataTypes>::activated(sofa::core::CollisionModel* cm) const
{
    return this->model->myActiver->activePoint(this->index, cm);
}

typedef ScytherPointCollisionModel<sofa::defaulttype::Vec3Types> ScytherPointModel;
typedef TScytherPoint<sofa::defaulttype::Vec3Types>              ScytherPoint;

////#if !defined(SCYTHER_POINT_COLLISION_MODEL_CPP)
extern template class SOFA_MESH_COLLISION_API ScytherPointCollisionModel<sofa::defaulttype::Vec3Types>;
////#endif

} // namespace SofaInterface

#endif
