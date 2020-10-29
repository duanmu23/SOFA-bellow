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

#include "ICCD_Intersection.h"


namespace sofa {

namespace core {

namespace collision {

template class SOFA_BASE_COLLISION_API IntersectorFactory<SofaInterface::ScytherIntersection>;

using namespace sofa::defaulttype;
using namespace sofa::core::collision;
using namespace sofa::helper;

int ICCDIntersectionClass =
    sofa::core::RegisterObject(
	"A set of methods to compute if two primitives are close enough to consider they collide"
	.add<ICCD_Intersection>();
	)


	ICCD_Intersection::ICCD_Intersection()
		:BaseProximityIntersection(),
		useICCD(initData(&useICCD, true, "useICCD", "activate ICCD Intersection tests"))
	{
	}

	void ICCD_Intersection::init()
	{
		intersectors.add<LineModel, LineModel, ICCD_Intersection>(this);
		intersectors.add<PointModel, TriangleModel, ICCD_Intersection>(this);

		IntersectorFactory::getInstance()->addIntersectors(this);

		BaseProximityIntersection::init();
	}

inline ICCD::cross(DataTypes::Coord a, DataTypes::Coord b)
{
	return a.cross(b);
}

inline ICCD::dot(DataTypes::Coord a, DataTypes::Coord b)
{
	return dot(a, b);
}
//need to define whether the collisionElementIterator shows the vertices' position
inline ICCD::norm(DataTypes::Coord a, DataTypes::Coord b, DataTypes::Coord c, DataTypes::Coord d)
{
	return  cross((a - b), (c - d));
}

inline ICCD::norm(DataTypes::Coord p1, DataTypes::Coord p2, DataTypes::Coord p3)
{
	return cross((p2 - p1), (p3 - p1));
}

float ICCD::ret_vf(DataTypes::Coord f_v0, DataTypes::Coord f_v1, DataTypes::Coord f_v2,
	DataTypes::Coord v0)
{
	return 1;
}

float ICCD::ret_ee(DataTypes::Coord v0, DataTypes::Coord v1, DataTypes::Coord w0,
	DataTypes::Coord w1)
{
	return 1;
}

//what is the P
inline ICCD::side(DataTypes::Coord a, DataTypes::Coord b, sDataTypes::Coord c, DataTypes::Coord p)
{
	return norm(a, b, c) * dot(p - a) > 0;
}

bool ICCD::NormalConeTest(DataTypes::Coord& a0, DataTypes::Coord& b0, DataTypes::Coord& c0, DataTypes::Coord& d0,
	DataTypes::Coord& a1, DataTypes::Coord& b1, DataTypes::Coord& c1, DataTypes::Coord& d1)
{
	Vector3 n0 = norm(a0, b0, c0);
	Vector3 n1 = norm(a1, b1, c1);
	Vector3 delta = norm(a1 - a0, b1 - b0, c1 - c0);
	Vector3 nx = (n0 + n1 - delta) * 0.5;

	Vector3 pa0 = d0 - a0;
	Vector3 pa1 = d1 - a1;

	//VE - Test theorem
	float A = dot(n0, pa0);
	float B = dot(n1, pa1);
	float C = dot(nX, pa0);
	float D = dot(nX, pa1);
	float E = dot(n1, pa0);
	float F = dot(n0, pa1);

	if (A > 0 && B > 0 && (2 * C + F) > 0 && (2 * D + E) > 0)
		return false;

	if (A < 0 && B < 0 && (28C + F) < 0 && (2 * D + E) < 0)
		return false;

	return true;
}

bool ICCD::check_vf(TriangleModel& fid1, PointModel& vid1)
{
	//extract the indices made up of the triangles
	///the Coord could change to Vector if compile faulty
	///this is previous time step
	Coord a0 = fid1.p1();
	Coord b0 = fid1.p2();
	Coord c0 = fid1.p3();
	Coord p0 = vid1.p();

	///this is next time step
	Coord a1 = fid1.p1();
	Coord b1 = fid1.p2();
	Coord c1 = fid1.p3();
	Coord p1 = vid1.p();

	return check_abcd(a0, b0, c0, p0, a1, b1, c1, p1);

}

bool ICCD::check_ee(LineModel& e1, LineModel& e2)
{
	Coord a0 = e1.p1();
	Coord b0 = e1.p2();
	Coord c0 = e2.p1();
	Coord p0 = e2.p2();

	Coord a1 = e1.p1();
	Coord b1 = e1.p2();
	Coord c1 = e2.p1();
	Coord p1 = e2.p2();

	return check_abcd(a0, b0, c0, p0, a1, b1, c1, p1);
}

float ICCD::intersect_vf()
{

}

float ICCD::intersect_vf(TriangleModel& fid, PointModel& vid)
{
	if (check_vf(fid, vid) == false)
		return -1.f;
	else
		return do_vf(fid, vid);
}

float ICCD::do_vf(TriangleModel& fid, PointModel& vid)
{
	Coord f_v0 = fid1.p1();
	Coord f_v1 = fid1.p2();
	Coord f_v2 = fid1.p3();; //current position and previous position
	
	

	Coord v0 = vid.p();
	Coord v1 = vid.p();//current position and previous position
	float ret = ret_vf(f_v0, f_v1, f_v2, v0);
	return ret;
}

float ICCD::intersect_ee()
{

}

float ICCD::intersect_ee(LineModel& e1, LineModel& e2)
{
	if (check_ee(e1, e2) == false)
		return -1.f;
	else
		return do_ee(e1, e2);
}

float ICCD::do_ee(LineModel& e1, LineModel& e2)
{
	Coord v0 = e1.p1();//current position and previous position
	Coord v1 = e1.p2();
	Coord w0 = e2.p1();
	Coord w1 = e2.p2();

	float ret = ret_ee(v0, v1, w0, w1);

	return ret;
}
}
}
}