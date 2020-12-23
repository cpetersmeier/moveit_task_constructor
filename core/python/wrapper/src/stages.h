#include "core.h"
#include <moveit/task_constructor/stages/current_state.h>
#include <moveit/task_constructor/stages/fixed_state.h>
#include <moveit/task_constructor/stages/fixed_cartesian_poses.h>
#include <moveit/task_constructor/stages/generate_pose.h>
#include <moveit/task_constructor/stages/generate_place_pose.h>
#include <moveit/task_constructor/stages/generate_grasp_pose.h>
#include <moveit/task_constructor/stages/fix_collision_objects.h>
#include <moveit/task_constructor/stages/modify_planning_scene.h>
#include <moveit/task_constructor/stages/move_relative.h>
#include <moveit/task_constructor/stages/move_to.h>
#include <moveit/task_constructor/stages/connect.h>
#include <moveit/task_constructor/stages/simple_grasp.h>
#include <moveit/task_constructor/stages/pick.h>
#include <moveit/task_constructor/stages/predicate_filter.h>
#include <moveit/task_constructor/stages/compute_ik.h>
#include <moveit/task_constructor/stages/passthrough.h>

namespace moveit {
namespace task_constructor {
namespace stages {

// Trampoline classes to allow inheritance in Python (overriding virtual functions)

template <class T = CurrentState>
class PyCurrentState : public PyGenerator<T>
{
public:
	using PyGenerator<T>::PyGenerator;
};

template <class T = FixedState>
class PyFixedState : public PyGenerator<T>
{
public:
	using PyGenerator<T>::PyGenerator;
};

template <class T = FixedCartesianPoses>
class PyFixedCartesianPoses : public PyMonitoringGenerator<T>
{
public:
	using PyMonitoringGenerator<T>::PyMonitoringGenerator;
};

template <class T = GeneratePose>
class PyGeneratePose : public PyMonitoringGenerator<T>
{
public:
	using PyMonitoringGenerator<T>::PyMonitoringGenerator;
};

template <class T = GeneratePlacePose>
class PyGeneratePlacePose : public PyGeneratePose<T>
{
public:
	using PyGeneratePose<T>::PyGeneratePose;
};

template <class T = GenerateGraspPose>
class PyGenerateGraspPose : public PyGeneratePose<T>
{
public:
	using PyGeneratePose<T>::PyGeneratePose;
};

template <class T = FixCollisionObjects>
class PyFixCollisionObjects : public PyPropagatingEitherWay<T>
{
public:
	using PyPropagatingEitherWay<T>::PyPropagatingEitherWay;
};

template <class T = ModifyPlanningScene>
class PyModifyPlanningScene : public PyPropagatingEitherWay<T>
{
public:
	using PyPropagatingEitherWay<T>::PyPropagatingEitherWay;
};

template <class T = MoveRelative>
class PyMoveRelative : public PyPropagatingEitherWay<T>
{
public:
	using PyPropagatingEitherWay<T>::PyPropagatingEitherWay;
};

template <class T = Connect>
class PyConnect : public PyConnecting<T>
{
public:
	using PyConnecting<T>::PyConnecting;
};

template <class T = SimpleGraspBase>
class PySimpleGraspBase : public PySerialContainer<T>
{
public:
	using PySerialContainer<T>::PySerialContainer;
};

template <class T = PickPlaceBase>
class PyPickPlaceBase : public PySerialContainer<T>
{
public:
	using PySerialContainer<T>::PySerialContainer;
};

template <class T = SimpleGrasp>
class PySimpleGrasp : public PySimpleGraspBase<T>
{
public:
	using PySimpleGraspBase<T>::PySimpleGraspBase;
};

template <class T = SimpleUnGrasp>
class PySimpleUnGrasp : public PySimpleGraspBase<T>
{
public:
	using PySimpleGraspBase<T>::PySimpleGraspBase;
};

template <class T = Pick>
class PyPick : public PyPickPlaceBase<T>
{
public:
	using PyPickPlaceBase<T>::PyPickPlaceBase;
};

template <class T = Place>
class PyPlace : public PyPickPlaceBase<T>
{
public:
	using PyPickPlaceBase<T>::PyPickPlaceBase;
};

template <class T = PredicateFilter>
class PyPredicateFilter : public PyWrapperBase<T>
{
public:
	using PyWrapperBase<T>::PyWrapperBase;
};

template <class T = ComputeIK>
class PyComputeIK : public PyWrapperBase<T>
{
public:
	using PyWrapperBase<T>::PyWrapperBase;
};

template <class T = PassThrough>
class PyPassThrough : public PyWrapperBase<T>
{
public:
	using PyWrapperBase<T>::PyWrapperBase;
};

}  // namespace stages
}  // namespace task_constructor
}  // namespace moveit