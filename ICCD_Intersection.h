//This function is to collect all the collect all the intersection function. Not containing other test parts
//The function including intersect between ee and vf

//how to identify
#ifndef ICCD_Intersection_H
#define ICCD_Intersection_H
#include "SofaGeneral/config.h"

#include <SofaBaseCollision/BaseProximityIntersection.h>
#include <SofaBaseCollision/MinProximityIntersection.h>

#include <SofaMeshCollision/MeshIntTool.h>
#include <sofa/helper/FnDispatcher.h>
#include <SofaBaseCollision/IntrUtility3.h>

#include <SofaBaseCollision/CubeModel.h>
#include <SofaMeshCollision/LineModel.h>
#include <SofaMeshCollision/PointModel.h>
#include <SofaMeshCollision/TriangleModel.h>
#include <SofaUserInteraction/RayModel.h>

#include "ICCD.h"


namespace sofa {

namespace component {

namespace collision {

	class SOFA_BASE_COLLISION_API ICCD_Intersection : public sofa::component::collision::BaseProximityIntersection
	{
		//	//should have some publich paramter here
		//public:
			//the input should be position
		template <class T>
		using Data = sofa::core::objectmodel::Data<T>;

		using CubeModel = sofa::component::collision::CubeModel;
		using PointModel = sofa::component::collision::PointModel;
		using LineModel = sofa::component::collision::LineModel;
		using TriangleModel = sofa::component::collision::TriangleModel;
		using RayModel = sofa::component::collision::RayModel;

		using Cube = sofa::component::collision::Cube;
		using Point = sofa::component::collision::Point;
		using Line = sofa::component::collision::Line;
		using Triangle = sofa::component::collision::Triangle;
		using Ray == sofa::component::collision::Ray;

	public:
		SOFA_CLASS(ICCD_Intersection, BaseProximityIntersection);

	protected:
		ICCD_Intersection();

	public:
		typedef sofa::core::collision::IntersectorFactory<ICCD_Intersection> IntersectorFactory;

		void init() override;

		float ret_vf();
		float ret_ee();

		bool ICCD::NormalConeTest();

		bool ICCD::check_vf();
		bool ICCD::check_vf();
		bool ICCD::check_ee();
		float ICCD::intersect_vf();
		float ICCD::intersect_vf();
		float ICCD::do_vf();
		float ICCD::intersect_ee();
		float ICCD::intersect_ee();
		float ICCD::do_ee();


		void beginBroadPhase() override;
		void endBroadPhase() override;

	};
}
}
#endif // !ICCD_Intersection_H

