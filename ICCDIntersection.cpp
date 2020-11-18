#define SOFA_COMPONENT_COLLISION_ICCDINTERSECTION_CPP
#include <sofa/helper/system/config.h>
#include <sofa/helper/proximity.h>
#include <sofa/helper/FnDispatcher.inl>

#include <sofa/core/ObjectFactory.h>
#include <sofa/core/collision/Intersection.inl>
#include <sofa/core/visual/VisualParams.h>
#include <sofa/core/collision/IntersectorFactory.h>

#include <sofa/defaulttype/Mat.h>
#include <sofa/defaulttype/Vec.h>

#include <SofaBaseCollision/ICCDIntersection.h>
#include <SofaBaseCollision/BaseIntTool.h>

// extra from MeshMinProximityIntersection
#include <SofaMeshCollision/MeshNewProximityIntersection.inl>

#include <algorithm>
#include <iostream>

namespace sofa
{

namespace core
{

   namespace collision
   {
      template class SOFA_BASE_COLLISION_API IntersectorFactory<component::collision::ICCDIntersection>;
   }
}

namespace component
{

namespace collision
{

//template class SOFA_BASE_COLLISION_API IntersectorFactory<SofaInterface::ICCDIntersection>;

using namespace sofa::defaulttype;
using namespace sofa::core::collision;
using namespace helper;

int ICCDIntersectionClass = sofa::core::RegisterObject("A set of methods to compute if two primitives are close enough to consider they collide")
        .add< ICCDIntersection >()
        ;

///it seems here not class declaration
ICCDIntersection::ICCDIntersection()
    :BaseProximityIntersection()
    ,useLineLine(initData(&useLineLine, false, "useLineLine", "Line-Line collision detection enabled"))
{
}

void ICCDIntersection::init()
{
        intersectors.add<LineModel, LineModel, ICCDIntersection>(this);
        intersectors.add<PointModel, TriangleModel, ICCDIntersection>(this);

        IntersectorFactory::getInstance()->addIntersectors(this);

        BaseProximityIntersection::init();
}

template <class DataTypes>
float ICCDIntersection::ret_vf(DataTypes& f_v0, DataTypes& f_v1, DataTypes& f_v2,
        DataTypes& v0)
{
        return 1.f;
}

template <class DataTypes>
float ICCDIntersection::ret_ee(DataTypes& v0, DataTypes& v1, DataTypes& w0,
        DataTypes& w1)
{
        return 1.f;
}

template <class DataTypes, class N>
bool ICCDIntersection::NormalConeTest(DataTypes& a0, DataTypes& b0, DataTypes& c0, DataTypes& d0,
        DataTypes& a1, DataTypes& b1, DataTypes& c1, DataTypes& d1)
{
        defaulttype::Vector3 n0 = norm(a0, b0, c0);
        defaulttype::Vector3 n1 = norm(a1, b1, c1);
        defaulttype::Vector3 delta = norm(a1 - a0, b1 - b0, c1 - c0);
        defaulttype::Vector3 nx = (n0 + n1 - delta) * 0.5;

        defaulttype::Vector3 pa0 = d0 - a0;
        defaulttype::Vector3 pa1 = d1 - a1;

	//VE - Test theorem
        Vector3 A = Vec<N,DataTypes>::dot(n0, pa0);
        Vector3 B = Vec<N,DataTypes>::dot(n1, pa1);
        Vector3 C = Vec<N,DataTypes>::dot(nx, pa0);
        Vector3 D = Vec<N,DataTypes>::dot(nx, pa1);
        Vector3 E = Vec<N,DataTypes>::dot(n1, pa0);
        Vector3 F = Vec<N,DataTypes>::dot(n0, pa1);

	if (A > 0 && B > 0 && (2 * C + F) > 0 && (2 * D + E) > 0)
		return false;

        if (A < 0 && B < 0 && (2 * C + F) < 0 && (2 * D + E) < 0)
		return false;

	return true;
}

bool ICCDIntersection::check_vf(Triangle& fid1, Point& vid1)
{
	//extract the indices made up of the triangles
	///the Coord could change to Vector if compile faulty
	///this is previous time step
        defaulttype::Vector3 a0 = fid1.p1();
        defaulttype::Vector3 b0 = fid1.p2();
        defaulttype::Vector3 c0 = fid1.p3();
        defaulttype::Vector3 p0 = vid1.p();

	///this is next time step
        defaulttype::Vector3 a1 = fid1.p1();
        defaulttype::Vector3 b1 = fid1.p2();
        defaulttype::Vector3 c1 = fid1.p3();
        defaulttype::Vector3 p1 = vid1.p();

        return NormalConeTest(a0, b0, c0, p0, a1, b1, c1, p1);

}

bool ICCDIntersection::check_ee(Line& e1, Line& e2)
{
        defaulttype::Vector3 a0 = e1.p1();
        defaulttype::Vector3 b0 = e1.p2();
        defaulttype::Vector3 c0 = e2.p1();
        defaulttype::Vector3 p0 = e2.p2();

        defaulttype::Vector3 a1 = e1.p1();
        defaulttype::Vector3 b1 = e1.p2();
        defaulttype::Vector3 c1 = e2.p1();
        defaulttype::Vector3 p1 = e2.p2();

        return NormalConeTest(a0, b0, c0, p0, a1, b1, c1, p1);
}

//float ICCD::intersect_vf()
//{

//}

float ICCDIntersection::intersect_vf(Triangle& fid, Point& vid)
{
	if (check_vf(fid, vid) == false)
		return -1.f;
	else
		return do_vf(fid, vid);
}

float ICCDIntersection::do_vf(Triangle& fid, Point& vid)
{
        defaulttype::Vector3 f_v0 = fid.p1();
        defaulttype::Vector3 f_v1 = fid.p2();
        defaulttype::Vector3 f_v2 = fid.p3(); //current position and previous position

        defaulttype::Vector3 v0 = vid.p();
        defaulttype::Vector3 v1 = vid.p();//current position and previous position
	float ret = ret_vf(f_v0, f_v1, f_v2, v0);
	return ret;
}

//float ICCD::intersect_ee()
//{

//}

float ICCDIntersection::intersect_ee(Line& e1, Line& e2)
{
	if (check_ee(e1, e2) == false)
		return -1.f;
	else
		return do_ee(e1, e2);
}

float ICCDIntersection::do_ee(Line& e1, Line& e2)
{
        defaulttype::Vector3 v0 = e1.p1();//current position and previous position
        defaulttype::Vector3 v1 = e1.p2();
        defaulttype::Vector3 w0 = e2.p1();
        defaulttype::Vector3 w1 = e2.p2();

	float ret = ret_ee(v0, v1, w0, w1);

	return ret;
}

} //namespace collision
} //namespace component
} //namespace sofa

