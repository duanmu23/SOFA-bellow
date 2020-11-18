#include <algorithm>
#include <cmath>
#include <sofa/core/ObjectFactory.h>
#include <sofa/core/visual/VisualParams.h>
#include <sofa/simulation/Simulation.h>

#include "ScytherCubeModel.h"

namespace SofaInterface {

using namespace sofa::defaulttype;

int ScytherCubeModelClass =
    sofa::core::RegisterObject("Collision model representing a cube, for Scyther collision detection")
        .add<ScytherCubeModel>();

ScytherCubeModel::ScytherCubeModel()
{
    enum_type = AABB_TYPE;
}

void ScytherCubeModel::resize(int size)
{
    int size0 = this->size;
    if (size == size0)
        return;
    // reset parent
    CollisionModel* parent = getPrevious();
    while (parent != NULL)
    {
        parent->resize(0);
        parent = parent->getPrevious();
    }
    this->sofa::core::CollisionModel::resize(size);
    this->elems.resize(size);
    this->parentOf.resize(size);
    // set additional indices
    for (int i = size0; i < size; ++i)
    {
        this->elems[i].children.first  = sofa::core::CollisionElementIterator(getNext(), i);
        this->elems[i].children.second = sofa::core::CollisionElementIterator(getNext(), i + 1);
        this->parentOf[i]              = i;
    }
}

void ScytherCubeModel::setParentOf(int childIndex, const Vector3& min, const Vector3& max)
{
    int i            = parentOf[childIndex];
    elems[i].minBBox = min;
    elems[i].maxBBox = max;
}

void ScytherCubeModel::setLeafCube(int cubeIndex, int childIndex)
{
    parentOf[childIndex]                   = cubeIndex;
    this->elems[cubeIndex].children.first  = sofa::core::CollisionElementIterator(getNext(), childIndex);
    this->elems[cubeIndex].children.second = sofa::core::CollisionElementIterator(getNext(), childIndex + 1);
    // elems[cubeIndex].minBBox = min;
    // elems[cubeIndex].maxBBox = max;
}

void ScytherCubeModel::setLeafCube(
    int cubeIndex, std::pair<sofa::core::CollisionElementIterator, sofa::core::CollisionElementIterator> children,
    const Vector3& min, const Vector3& max)
{
    elems[cubeIndex].minBBox  = min;
    elems[cubeIndex].maxBBox  = max;
    elems[cubeIndex].children = children;
}

int ScytherCubeModel::addCube(ScytherCube subcellsBegin, ScytherCube subcellsEnd)
{
    int i = size;
    this->sofa::core::CollisionModel::resize(size + 1);
    elems.resize(size + 1);
    // elems[i].subcells = std::make_pair(subcellsBegin, subcellsEnd);
    elems[i].subcells.first  = subcellsBegin;
    elems[i].subcells.second = subcellsEnd;
    elems[i].children.first  = sofa::core::CollisionElementIterator();
    elems[i].children.second = sofa::core::CollisionElementIterator();
    updateCube(i);
    return i;
}

void ScytherCubeModel::updateCube(int index)
{
    const std::pair<ScytherCube, ScytherCube>& subcells = elems[index].subcells;
    if (subcells.first != subcells.second)
    {
        ScytherCube c       = subcells.first;
        Vector3     minBBox = c.minVect();
        Vector3     maxBBox = c.maxVect();
        ++c;
        while (c != subcells.second)
        {
            const Vector3& cmin = c.minVect();
            const Vector3& cmax = c.maxVect();
            for (int j = 0; j < 3; j++)
            {
                if (cmax[j] > maxBBox[j])
                    maxBBox[j] = cmax[j];
                if (cmin[j] < minBBox[j])
                    minBBox[j] = cmin[j];
            }
            ++c;
        }
        elems[index].minBBox = minBBox;
        elems[index].maxBBox = maxBBox;
    }
}

void ScytherCubeModel::updateCubes()
{
    for (int i = 0; i < size; i++)
        updateCube(i);
}

void ScytherCubeModel::draw(const sofa::core::visual::VisualParams* vparams)
{
    if (!isActive() || !((getNext() == NULL) ? vparams->displayFlags().getShowCollisionModels()
                                             : vparams->displayFlags().getShowBoundingCollisionModels()))
        return;

    int             level = 0;
    CollisionModel* m     = getPrevious();
    float           color = 1.0f;
    while (m != NULL)
    {
        m = m->getPrevious();
        ++level;
        color *= 0.5f;
    }
    Vec<4, float> c;
    if (isSimulated())
        c = Vec<4, float>(1.0f, 1.0f, 1.0f, color);
    else
        c = Vec<4, float>(1.0f, 1.0f, 1.0f, color);

    std::vector<Vector3> points;
    for (int i = 0; i < size; i++)
    {
        const Vector3& vmin = elems[i].minBBox;
        const Vector3& vmax = elems[i].maxBBox;

        points.push_back(Vector3(vmin[0], vmin[1], vmin[2]));
        points.push_back(Vector3(vmin[0], vmin[1], vmax[2]));
        points.push_back(Vector3(vmin[0], vmax[1], vmin[2]));
        points.push_back(Vector3(vmin[0], vmax[1], vmax[2]));
        points.push_back(Vector3(vmax[0], vmin[1], vmin[2]));
        points.push_back(Vector3(vmax[0], vmin[1], vmax[2]));
        points.push_back(Vector3(vmax[0], vmax[1], vmin[2]));
        points.push_back(Vector3(vmax[0], vmax[1], vmax[2]));

        points.push_back(Vector3(vmin[0], vmin[1], vmin[2]));
        points.push_back(Vector3(vmin[0], vmax[1], vmin[2]));
        points.push_back(Vector3(vmin[0], vmin[1], vmax[2]));
        points.push_back(Vector3(vmin[0], vmax[1], vmax[2]));
        points.push_back(Vector3(vmax[0], vmin[1], vmin[2]));
        points.push_back(Vector3(vmax[0], vmax[1], vmin[2]));
        points.push_back(Vector3(vmax[0], vmin[1], vmax[2]));
        points.push_back(Vector3(vmax[0], vmax[1], vmax[2]));

        points.push_back(Vector3(vmin[0], vmin[1], vmin[2]));
        points.push_back(Vector3(vmax[0], vmin[1], vmin[2]));
        points.push_back(Vector3(vmin[0], vmax[1], vmin[2]));
        points.push_back(Vector3(vmax[0], vmax[1], vmin[2]));
        points.push_back(Vector3(vmin[0], vmin[1], vmax[2]));
        points.push_back(Vector3(vmax[0], vmin[1], vmax[2]));
        points.push_back(Vector3(vmin[0], vmax[1], vmax[2]));
        points.push_back(Vector3(vmax[0], vmax[1], vmax[2]));

        //        std::cout << i << "   " << vmin[0] << ", " << vmin[1] << ", " << vmin[2] << std::endl;
        //        std::cout << i << "   " << vmax[0] << ", " << vmax[1] << ", " << vmax[2] << std::endl;
    }

    vparams->drawTool()->drawLines(points, 1, Vec<4, float>(c));

    if (getPrevious() != NULL)
        getPrevious()->draw(vparams);
}

std::pair<sofa::core::CollisionElementIterator, sofa::core::CollisionElementIterator>
ScytherCubeModel::getInternalChildren(int index) const
{
    return elems[index].subcells;
}

std::pair<sofa::core::CollisionElementIterator, sofa::core::CollisionElementIterator>
ScytherCubeModel::getExternalChildren(int index) const
{
    return elems[index].children;
    /*
        sofa::core::CollisionElementIterator i1 = elems[index].leaf;
        if (!i1.valid())
        {
            return std::make_pair(sofa::core::CollisionElementIterator(),sofa::core::CollisionElementIterator());
        }
        else
        {
            sofa::core::CollisionElementIterator i2 = i1; ++i2;
            return std::make_pair(i1,i2);
        }
    */
}

bool ScytherCubeModel::isLeaf(int index) const
{
    return elems[index].children.first.valid();
}

void ScytherCubeModel::computeBoundingTree(int maxDepth)
{
    //    if(maxDepth <= 0)
    //        return;

    maxDepth = 0;

    // sout << ">ScytherCubeModel::computeBoundingTree(" << maxDepth << ")" << sendl;
    std::list<ScytherCubeModel*> levels;
    levels.push_front(createPrevious<ScytherCubeModel>());
    // for (int i = 0; i < maxDepth; i++)
    //    levels.push_front(levels.front()->createPrevious<ScytherCubeModel>());
    ScytherCubeModel* root = levels.front();
    // if (isStatic() && root->getPrevious() == NULL && !root->empty()) return; // No need to recompute BBox if immobile

    //    std::cout << root->getClassName() << "  " << root->getPathName() << std::endl;
    //    std::cout << "    " << root->getNumberCells() << "  " << root->getSize() << std::endl;

    if (root->empty() || root->getPrevious() != NULL)
    {
        // Tree must be reconstructed
        // sout << "Building Tree with depth "<<maxDepth<<" from "<<size<<" elements."<<sendl;
        // First remove extra levels
        while (root->getPrevious() != NULL)
        {
            sofa::core::CollisionModel::SPtr m = root->getPrevious();
            root->setPrevious(m->getPrevious());
            if (m->getMaster())
                m->getMaster()->removeSlave(m);
            // delete m;
            m.reset();
        }
        // Then clear all existing levels
        {
            for (std::list<ScytherCubeModel*>::iterator it = levels.begin(); it != levels.end(); ++it)
                (*it)->resize(0);
        }
        // Then build root cell
        // sout << "ScytherCubeModel: add root cube"<<sendl;
        root->addCube(ScytherCube(this, 0), ScytherCube(this, size));
        // Construct tree by splitting cells along their biggest dimension
        std::list<ScytherCubeModel*>::iterator it    = levels.begin();
        ScytherCubeModel*                      level = *it;
        ++it;
        int lvl = 0;
        while (it != levels.end())
        {
            // sout << "ScytherCubeModel: split level "<<lvl<<sendl;
            ScytherCubeModel* clevel = *it;
            clevel->elems.reserve(level->size * 2);
            for (ScytherCube cell = ScytherCube(level->begin()); level->end() != cell; ++cell)
            {
                const std::pair<ScytherCube, ScytherCube>& subcells = cell.subcells();
                int ncells = subcells.second.getIndex() - subcells.first.getIndex();
                // sout << "ScytherCubeModel: level "<<lvl<<" cell "<<cell.getIndex()<<": current subcells
                // "<<subcells.first.getIndex() << " - "<<subcells.second.getIndex()<<sendl;
                if (ncells > 4)
                {
                    // Only split cells with more than 4 childs
                    // Find the biggest dimension
                    int     splitAxis;
                    Vector3 l      = cell.maxVect() - cell.minVect();
                    int     middle = subcells.first.getIndex() + (ncells + 1) / 2;
                    if (l[0] > l[1])
                        if (l[0] > l[2])
                            splitAxis = 0;
                        else
                            splitAxis = 2;
                    else if (l[1] > l[2])
                        splitAxis = 1;
                    else
                        splitAxis = 2;

                        // Separate cells on each side of the median cell

#if defined(__GNUC__) && (__GNUC__ == 4)
                    // && (__GNUC_MINOR__ == 1) && (__GNUC_PATCHLEVEL__ == 1)
                    // there is apparently a bug in std::sort with GCC 4.x
                    if (splitAxis == 0)
                        qsort(
                            &(elems[subcells.first.getIndex()]), subcells.second.getIndex() - subcells.first.getIndex(),
                            sizeof(elems[0]), CubeSortPredicate::sortCube<0>);
                    else if (splitAxis == 1)
                        qsort(
                            &(elems[subcells.first.getIndex()]), subcells.second.getIndex() - subcells.first.getIndex(),
                            sizeof(elems[0]), CubeSortPredicate::sortCube<1>);
                    else
                        qsort(
                            &(elems[subcells.first.getIndex()]), subcells.second.getIndex() - subcells.first.getIndex(),
                            sizeof(elems[0]), CubeSortPredicate::sortCube<2>);
#else
                    CubeSortPredicate sortpred(splitAxis);
                    // std::nth_element(elems.begin()+subcells.first.getIndex(),elems.begin()+middle,elems.begin()+subcells.second.getIndex(),
                    // sortpred);
                    std::sort(
                        elems.begin() + subcells.first.getIndex(), elems.begin() + subcells.second.getIndex(),
                        sortpred);
#endif

                    // Create the two new subcells
                    ScytherCube cmiddle(this, middle);
                    int         c1 = clevel->addCube(subcells.first, cmiddle);
                    int         c2 = clevel->addCube(cmiddle, subcells.second);
                    // sout << "L"<<lvl<<" cell "<<cell.getIndex()<<" split along
                    // "<<(splitAxis==0?'X':splitAxis==1?'Y':'Z')<<" in cell "<<c1<<" size
                    // "<<middle-subcells.first.getIndex()<<" and cell "<<c2<<" size
                    // "<<subcells.second.getIndex()-middle<<"."<<sendl; level->elems[cell.getIndex()].subcells =
                    // std::make_pair(ScytherCube(clevel,c1),ScytherCube(clevel,c2+1));
                    level->elems[cell.getIndex()].subcells.first  = ScytherCube(clevel, c1);
                    level->elems[cell.getIndex()].subcells.second = ScytherCube(clevel, c2 + 1);
                }
            }
            ++it;
            level = clevel;
            ++lvl;
        }
        if (!parentOf.empty())
        {
            // Finally update parentOf to reflect new cell order
            for (int i = 0; i < size; i++)
                parentOf[elems[i].children.first.getIndex()] = i;
        }
    }
    else
    {
        // Simply update the existing tree, starting from the bottom
        int lvl = 0;
        for (std::list<ScytherCubeModel*>::reverse_iterator it = levels.rbegin(); it != levels.rend(); ++it)
        {
            // sout << "ScytherCubeModel: update level "<<lvl<<sendl;
            (*it)->updateCubes();
            ++lvl;
        }
    }
    // sout << "<ScytherCubeModel::computeBoundingTree("<<maxDepth<<")"<<sendl;
}

} // namespace SofaInterface
