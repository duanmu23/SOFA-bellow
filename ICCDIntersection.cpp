#include <SofaBaseCollision/ICCDIntersection.h>
#include <sofa/helper/config.h>

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


namespace sofa {

namespace component {

namespace collision {

//template class SOFA_BASE_COLLISION_API IntersectorFactory<SofaInterface::ICCDIntersection>;

using namespace sofa::defaulttype;
using namespace sofa::core::collision;
using namespace sofa::helper;

///it seems here not class declaration
ICCDIntersection()
{
}

int ICCDIntersectionClass =
    sofa::core::RegisterObject(
        "A set of methods to compute if two primitives are close enough to consider they collide").add< ICCDIntersection >();

void ICCDIntersection::init()
{
        intersectors.add<LineModel, LineModel, ICCDIntersection>(this);
        intersectors.add<PointModel, TriangleModel, ICCDIntersection>(this);

        IntersectorFactory::getInstance()->addIntersectors(this);

        BaseProximityIntersection::init();
}

float ICCDIntersection::ret_vf(VecCoord& f_v0, VecCoord& f_v1, VecCoord& f_v2,
        VecCoord& v0)
{
	return 1;
}

float ICCDIntersection::ret_ee(VecCoord& v0, VecCoord& v1, VecCoord& w0,
        VecCoord& w1)
{
	return 1;
}

bool ICCDIntersection::NormalConeTest(VecCoord& a0, VecCoord& b0, VecCoord& c0, VecCoord& d0,
        VecCoord& a1, VecCoord& b1, VecCoord& c1, VecCoord& d1)
{
        VecCoord n0 = norm(a0, b0, c0);
        VecCoord n1 = norm(a1, b1, c1);
        VecCoord delta = norm(a1 - a0, b1 - b0, c1 - c0);
        VecCoord nx = (n0 + n1 - delta) * 0.5;

        VecCoord pa0 = d0 - a0;
        VecCoord pa1 = d1 - a1;

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

bool ICCDIntersection::check_vf(TriangleCollisionModel& fid1, PointCollisionModel& vid1)
{
	//extract the indices made up of the triangles
	///the Coord could change to Vector if compile faulty
	///this is previous time step
        VecCoord a0 = fid1.p1();
        VecCoord b0 = fid1.p2();
        VecCoord c0 = fid1.p3();
        VecCoord p0 = vid1.p();

	///this is next time step
        VecCoord a1 = fid1.p1();
        VecCoord b1 = fid1.p2();
        VecCoord c1 = fid1.p3();
        VecCoord p1 = vid1.p();

	return check_abcd(a0, b0, c0, p0, a1, b1, c1, p1);

}

bool ICCDIntersection::check_ee(LineCollisionModel& e1, LineCollisionModel& e2)
{
        VecCoord a0 = e1.p1();
        VecCoord b0 = e1.p2();
        VecCoord c0 = e2.p1();
        VecCoord p0 = e2.p2();

        VecCoord a1 = e1.p1();
        VecCoord b1 = e1.p2();
        VecCoord c1 = e2.p1();
        VecCoord p1 = e2.p2();

	return check_abcd(a0, b0, c0, p0, a1, b1, c1, p1);
}

//float ICCD::intersect_vf()
//{

//}

float ICCDIntersection::intersect_vf(TriangleCollisionModel& fid, PointCollisionModel& vid)
{
	if (check_vf(fid, vid) == false)
		return -1.f;
	else
		return do_vf(fid, vid);
}

float ICCDIntersection::do_vf(TriangleCollisionModel& fid, PointCollisionModel& vid)
{
        VecCoord f_v0 = fid1.p1();
        VecCoord f_v1 = fid1.p2();
        VecCoord f_v2 = fid1.p3(); //current position and previous position

        VecCoord v0 = vid.p();
        VecCoord v1 = vid.p();//current position and previous position
	float ret = ret_vf(f_v0, f_v1, f_v2, v0);
	return ret;
}

//float ICCD::intersect_ee()
//{

//}

float ICCDIntersection::intersect_ee(LineCollisionModel& e1, LineCollisionModel& e2)
{
	if (check_ee(e1, e2) == false)
		return -1.f;
	else
		return do_ee(e1, e2);
}

float ICCDIntersection::do_ee(LineCollisionModel& e1, LineCollisionModel& e2)
{
        VecCoord v0 = e1.p1();//current position and previous position
        VecCoord v1 = e1.p2();
        VecCoord w0 = e2.p1();
        VecCoord w1 = e2.p2();

	float ret = ret_ee(v0, v1, w0, w1);

	return ret;
}
}
}
}

