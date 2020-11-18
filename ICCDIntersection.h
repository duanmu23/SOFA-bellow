//This function is to collect all the collect all the intersection function. Not containing other test parts
//The function including intersect between ee and vf

//
#ifndef ICCDIntersection_H
#define ICCDIntersection_H

#include <iostream>
#include <algorithm>
#include <memory>

#include <sofa/helper/config.h>
#include <sofa/helper/FnDispatcher.h>
#include <sofa/helper/vector.h>


#include <SofaBaseCollision/BaseProximityIntersection.h>
#include <SofaBaseCollision/MinProximityIntersection.h>

#include <SofaMeshCollision/MeshIntTool.h>
#include <SofaBaseCollision/IntrUtility3.h>
#include <SofaBaseCollision/CubeModel.h>
#include <SofaMeshCollision/LineModel.h>
#include <SofaMeshCollision/PointModel.h>
#include <SofaMeshCollision/TriangleModel.h>

#include <sofa/core/CollisionModel.h>

#include <sofa/defaulttype/VecTypes.h>
#include <sofa/defaulttype/Mat.h>
#include <sofa/defaulttype/Vec.h>

//#include "ICCD.h"


namespace sofa {

namespace component {

namespace collision {

class SOFA_BASE_COLLISION_API ICCDIntersection :
        public sofa::component::collision::BaseProximityIntersection
{
		//	//should have some publich paramter here
		//public:
			//the input should be position
//        using Data = sofa::core::objectmodel::Data<T>;
//        using CubeModel =  sofa::component::collision::CubeCollisionModel;
//        using PointModel = sofa::component::collision::PointCollisionModel;
//        using LineModel = sofa::component::collision::LineCollisionModel;
//        using TriangleModel = sofa::component::collision::TriangleCollisionModel;


//        using Cube = sofa::component::collision::Cube;
//        using Point = sofa::component::collision::Point;
//        using Line = sofa::component::collision::Line;
//        using Triangle = sofa::component::collision::Triangle;

//		using Ray == sofa::component::collision::Ray;

public:
        SOFA_CLASS(ICCDIntersection, BaseProximityIntersection);
        Data<bool> useLineLine;

protected:
        ICCDIntersection();
public:
        typedef sofa::core::collision::IntersectorFactory<ICCDIntersection> IntersectorFactory;

        void init() override;

        //need to define whether the collisionElementIterator shows the vertices' position
        template <class DataTypes , class N>
        inline float ICCDIntersection::norm(DataTypes a, DataTypes b, DataTypes c, DataTypes d){ return  defaulttype::Vec<N, DataTypes>::cross((a - b), (c - d)); }

        template <class DataTypes, class N>
        inline float ICCDIntersection::norm(DataTypes p1, DataTypes p2, DataTypes p3){ return defaulttype::Vec<N, DataTypes>::cross((p2 - p1), (p3 - p1)); }

        //what is the P
        template <class DataTypes, class N>
        inline bool ICCDIntersection::side(DataTypes a, DataTypes b, DataTypes c, DataTypes p){ return norm(a, b, c) * defaulttype::Vec<N, DataTypes>::dot(p, a) > 0; }

        float ret_vf();
        float ret_ee();

        template <class DataTypes, , class N>
        bool ICCDIntersection::NormalConeTest(DataTypes&, DataTypes&, DataTypes&, DataTypes&,
                                                       DataTypes&, DataTypes&, DataTypes&, DataTypes&);


        bool ICCDIntersection::check_vf(Triangle&, Point&);
        bool ICCDIntersection::check_ee(Line&, Line&);
        float ICCDIntersection::intersect_vf(Triangle&, Point&);
        float ICCDIntersection::do_vf(Triangle&, Point&);
        float ICCDIntersection::intersect_ee(Line&, Line&);
        float ICCDIntersection::do_ee(Line&, Line&);

        void beginBroadPhase() override;
        void endBroadPhase() override;

};

} //namespace collision

} //namespace component

namespace core
{
namespace collision
{
#if  !defined(SOFA_COMPONENT_COLLISION_ICCDIntersection_CPP)
extern template class SOFA_BASE_COLLISION_API IntersectorFactory<component::collision::ICCDIntersection>;
#endif
}
}

} //namespace sofa
#endif // !ICCDIntersection_H

