
#ifndef SCYTHER_COLLISION_CUBE_MODEL_H
#define SCYTHER_COLLISION_CUBE_MODEL_H
#include "SofaBaseCollision/config.h"

#include <sofa/core/CollisionModel.h>
#include <SofaBaseMechanics/MechanicalObject.h>
#include <sofa/defaulttype/VecTypes.h>


namespace SofaInterface
{

class ScytherCubeModel;

class ScytherCube : public sofa::core::TCollisionElementIterator<ScytherCubeModel>
{
public:
    ScytherCube(ScytherCubeModel* model=nullptr, int index=0);

    explicit ScytherCube(const sofa::core::CollisionElementIterator& i);

    const sofa::defaulttype::Vector3& minVect() const;

    const sofa::defaulttype::Vector3& maxVect() const;

    const std::pair<ScytherCube,ScytherCube>& subcells() const;
};



class SOFA_BASE_COLLISION_API ScytherCubeModel : public sofa::core::CollisionModel
{
public:
    SOFA_CLASS(ScytherCubeModel,sofa::core::CollisionModel);

    struct CubeData
    {
        sofa::defaulttype::Vector3 minBBox, maxBBox;
        std::pair<ScytherCube,ScytherCube> subcells;
        std::pair<sofa::core::CollisionElementIterator,sofa::core::CollisionElementIterator> children; ///< Note that children is only meaningfull if subcells in empty
    };

    class CubeSortPredicate
    {
        int axis;
    public:
        CubeSortPredicate(int axis) : axis(axis) {}
        bool operator()(const CubeData& c1,const CubeData& c2) const
        {
            SReal v1 = c1.minBBox[axis]+c1.maxBBox[axis];
            SReal v2 = c2.minBBox[axis]+c2.maxBBox[axis];
            return v1 < v2;
        }
        template<int Axis>
        static int sortCube(const void* p1, const void* p2)
        {
            const ScytherCubeModel::CubeData* c1 = (const ScytherCubeModel::CubeData*)p1;
            const ScytherCubeModel::CubeData* c2 = (const ScytherCubeModel::CubeData*)p2;
            SReal v1 = c1->minBBox[Axis] + c1->maxBBox[Axis];
            SReal v2 = c2->minBBox[Axis] + c2->maxBBox[Axis];

            if (v1 < v2)
                return -1;
            else if (v1 > v2)
                return 1;
            else
                return 0;
        }
    };

protected:
    sofa::helper::vector<CubeData> elems;
    sofa::helper::vector<int> parentOf; ///< Given the index of a child leaf element, store the index of the parent cube

public:
    typedef sofa::core::CollisionElementIterator ChildIterator;
    typedef sofa::defaulttype::Vec3Types DataTypes;
    typedef ScytherCube Element;
    friend class ScytherCube;
protected:
    ScytherCubeModel();
public:
    void resize(int size) override;

    void setParentOf(int childIndex, const sofa::defaulttype::Vector3& min, const sofa::defaulttype::Vector3& max);
    void setLeafCube(int cubeIndex, int childIndex);
    void setLeafCube(int cubeIndex, std::pair<sofa::core::CollisionElementIterator,sofa::core::CollisionElementIterator> children, const sofa::defaulttype::Vector3& min, const sofa::defaulttype::Vector3& max);


    unsigned int getNumberCells() { return (unsigned int)elems.size();}

    void getBoundingTree ( sofa::helper::vector< std::pair< sofa::defaulttype::Vector3, sofa::defaulttype::Vector3> > &bounding )
    {
        bounding.resize(elems.size());
        for (unsigned int index=0; index<elems.size(); index++)
        {
            bounding[index] = std::make_pair( elems[index].minBBox, elems[index].maxBBox);
        }
    }

    int getLeafIndex(int index) const
    {
        return elems[index].children.first.getIndex();
    }

    int getLeafEndIndex(int index) const
    {
        return elems[index].children.second.getIndex();
    }

    const CubeData & getCubeData(int index)const{return elems[index];}

    // -- CollisionModel interface

    /**
      *Here we make up the hierarchy (a tree) of bounding boxes which contain final CollisionElements like Spheres or Triangles.
      *The leafs of the tree contain final CollisionElements. This hierarchy is made up from the top to the bottom, i.e., we begin
      *to compute a bounding box containing all CollisionElements, then we divide this big bounding box into two boxes.
      *These new two boxes inherit from the root box and have depth 1. Then we can do the same operation for the new boxes.
      *The division is done only if the box contains more than 4 final CollisionElements and if the depth doesn't exceed
      *the max depth. The division is made along an axis. This axis corresponds to the biggest dimension of the current bounding box.
      *Note : a bounding box is a Cube here.
      */
    void computeBoundingTree(int maxDepth=0) override;

    std::pair<sofa::core::CollisionElementIterator,sofa::core::CollisionElementIterator> getInternalChildren(int index) const override;

    std::pair<sofa::core::CollisionElementIterator,sofa::core::CollisionElementIterator> getExternalChildren(int index) const override;

    bool isLeaf( int index ) const override;

    void draw(const sofa::core::visual::VisualParams* vparams) override;

    int addCube(ScytherCube subcellsBegin, ScytherCube subcellsEnd);
    void updateCube(int index);
    void updateCubes();
};





inline ScytherCube::ScytherCube(ScytherCubeModel* model, int index)
    : sofa::core::TCollisionElementIterator<ScytherCubeModel>(model, index)
{}

inline ScytherCube::ScytherCube(const sofa::core::CollisionElementIterator& i)
    : sofa::core::TCollisionElementIterator<ScytherCubeModel>(static_cast<ScytherCubeModel*>(i.getCollisionModel()), i.getIndex())
{
}

inline const sofa::defaulttype::Vector3& ScytherCube::minVect() const
{
    return model->elems[index].minBBox;
}

inline const sofa::defaulttype::Vector3& ScytherCube::maxVect() const
{
    return model->elems[index].maxBBox;
}


inline const std::pair<ScytherCube,ScytherCube>& ScytherCube::subcells() const
{
    return model->elems[index].subcells;
}





} // namespace SofaInterface


#endif
