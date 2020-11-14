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


//#include "ICCD.h"


namespace sofa {

namespace component {

namespace collision {

template <class TDataTypes>

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

protected:
        ICCDIntersection();
        ~ICCDIntersection() override;

public:
        typedef sofa::core::collision::IntersectorFactory<ICCDIntersection> IntersectorFactory;


        typedef TDataTypes DataTypes;
        typedef typename DataTypes::Coord VecCoord;

        void init() override;
        inline float ICCDIntersection::cross(VecCoord& a, VecCoord& b){ return a.cross(b); }

        inline float ICCDIntersection::dot(VecCoord& a, VecCoord& b){ return dot(a, b); }

        //need to define whether the collisionElementIterator shows the vertices' position
        inline float ICCDIntersection::norm(VecCoord& a, VecCoord& b, VecCoord& c, VecCoord& d){ return  cross((a - b), (c - d)); }

        inline float ICCDIntersection::norm(VecCoord& p1, VecCoord& p2, VecCoord& p3){ return cross((p2 - p1), (p3 - p1)); }

        //what is the P
        inline bool ICCDIntersection::side(VecCoord& a, VecCoord& b, VecCoord& c, VecCoord& p){ return norm(a, b, c) * dot(p - a) > 0; }

        float ret_vf();
        float ret_ee();

        bool ICCDIntersection::NormalConeTest(VecCoord&, VecCoord&, VecCoord&, VecCoord&,
                                                       VecCoord&, VecCoord&, VecCoord&, VecCoord&);

        bool ICCDIntersection::check_vf(TriangleCollisionModel&, PointCollisionModel&);
        bool ICCDIntersection::check_ee(LineCollisionModel&, LineCollsionModel&);
        float ICCDIntersection::intersect_vf(TriangleCollisionModel&, PointCollisionModel&);
        float ICCDIntersection::do_vf(TriangleCollisionModel&, PointCollisionModel&);
        float ICCDIntersection::intersect_ee(LineCollisionModel& e1, LineCollisionModel& e2);
        float ICCDIntersection::do_ee(LineCollisionModel& e1, LineCollisionModel& e2);

        void beginBroadPhase() override;
        void endBroadPhase() override;

};
}
}
}
#endif // !ICCDIntersection_H

