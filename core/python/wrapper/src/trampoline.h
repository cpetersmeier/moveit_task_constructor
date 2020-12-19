#include <moveit/task_constructor/stage.h>
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
#include <moveit/task_constructor/task.h>
#include <moveit/task_constructor/stages/compute_ik.h>
#include <moveit/task_constructor/stages/passthrough.h>

namespace moveit {
namespace task_constructor {

// Trampoline classes to allow inheritance in Python (overriding virtual functions)
template <class T = Stage>
class PyStage : public T
{
public:
	using T::T;

	void init(const moveit::core::RobotModelConstPtr& robot_model) override {
		PYBIND11_OVERRIDE(void, T, init, robot_model);
	}
	void reset() override { PYBIND11_OVERRIDE(void, T, reset, ); }
};

template <class T = Generator>
class PyGenerator : public PyStage<T>
{
public:
	using PyStage<T>::PyStage;

	bool canCompute() const override { PYBIND11_OVERRIDE_PURE(bool, T, canCompute, ); }
	void compute() override { PYBIND11_OVERRIDE_PURE(void, T, compute, ); }
};

template <class T = MonitoringGenerator>
class PyMonitoringGenerator : public PyGenerator<T>
{
public:
	using PyGenerator<T>::PyGenerator;
	void onNewSolution(const SolutionBase& s) override { PYBIND11_OVERRIDE_PURE(void, T, onNewSolution, s); }
};

template<class T = PropagatingEitherWay>
class PyPropagatingEitherWay : public  PyStage<T>
{
public:
	using PyStage<T>::PyStage;
};

template<class T = PropagatingBackward>
class PyPropagatingBackward : public PyPropagatingEitherWay<T>
{
public:
	using PyPropagatingEitherWay<T>::PyPropagatingEitherWay;
};

template<class T = PropagatingForward>
class PyPropagatingForward : public PyPropagatingEitherWay<T>
{
public:
	using PyPropagatingEitherWay<T>::PyPropagatingEitherWay;
};

template<class T = Connecting>
class PyConnecting : public PyStage<T>
{
public:
	using PyStage<T>::PyStage;
	void compute(const InterfaceState& from, const InterfaceState& to) override
		{ PYBIND11_OVERRIDE_PURE(void, T, compute, from, to); }
};

template<class T = ContainerBase>
class PyContainerBase : public PyStage<T>
{
public:
	using PyStage<T>::PyStage;

	bool insert(Stage::pointer&& stage, int before = -1) override
		{ PYBIND11_OVERRIDE(bool, T, insert, stage, before); }

	Stage::pointer remove(int pos) override
		{ PYBIND11_OVERRIDE(Stage::pointer, T, remove, pos); }

	Stage::pointer remove(Stage* child) override
		{ PYBIND11_OVERRIDE(Stage::pointer, T, remove, child); }

	void clear() override { PYBIND11_OVERRIDE(void, T, clear, ); }

	bool canCompute() const override { PYBIND11_OVERRIDE_PURE(bool, T, canCompute, ); }

	void compute() override { PYBIND11_OVERRIDE_PURE(void, T, compute, ); }

	void onNewSolution(const SolutionBase& s) override
		{ PYBIND11_OVERRIDE_PURE(void, T, onNewSolution, s); }
};

template<class T = SerialContainer>
class PySerialContainer : public PyContainerBase<T>
{
public:
	using PyContainerBase<T>::PyContainerBase;
};

template<class T = ParallelContainerBase>
class PyParallelContainerBase : public PyContainerBase<T>
{
public:
	using PyContainerBase<T>::PyContainerBase;
};

template<class T = WrapperBase>
class PyWrapperBase : public PyParallelContainerBase<T>
{
public:
	using PyParallelContainerBase<T>::PyParallelContainerBase;
};

template<class T = Alternatives>
class PyAlternative : public PyParallelContainerBase<T>
{
public:
	using PyParallelContainerBase<T>::PyParallelContainerBase;
};

template<class T = Fallbacks>
class PyFallbacks : public PyParallelContainerBase<T>
{
public:
	using PyParallelContainerBase<T>::PyParallelContainerBase;
};

template<class T = Merger>
class PyMerger : public PyParallelContainerBase<T>
{
public:
	using PyParallelContainerBase<T>::PyParallelContainerBase;
};

template<class T = Task>
class PyTask : public PyWrapperBase<T>
{
public:
	using PyWrapperBase<T>::PyWrapperBase;
};

namespace stages {

template <class T = CurrentState>
class PyCurrentState : public PyGenerator<T>
{
public:
	using PyGenerator<T>::PyGenerator;
};

template<class T = FixedState>
class PyFixedState : public PyGenerator<T>
{
public:
	using PyGenerator<T>::PyGenerator;
};

template<class T = FixedCartesianPoses>
class PyFixedCartesianPoses : public PyMonitoringGenerator<T>
{
public:
	using PyMonitoringGenerator<T>::PyMonitoringGenerator;
};

template<class T = GeneratePose>
class PyGeneratePose : public PyMonitoringGenerator<T>
{
public:
	using PyMonitoringGenerator<T>::PyMonitoringGenerator;
};

template<class T = GeneratePlacePose>
class PyGeneratePlacePose : public PyGeneratePose<T>
{
public:
	using PyGeneratePose<T>::PyGeneratePose;
};

template<class T = GenerateGraspPose>
class PyGenerateGraspPose : public PyGeneratePose<T>
{
public:
	using PyGeneratePose<T>::PyGeneratePose;
};

template<class T = FixCollisionObjects>
class PyFixCollisionObjects : public PyPropagatingEitherWay<T>
{
public:
	using PyPropagatingEitherWay<T>::PyPropagatingEitherWay;
};

template<class T = ModifyPlanningScene>
class PyModifyPlanningScene : public PyPropagatingEitherWay<T>
{
public:
	using PyPropagatingEitherWay<T>::PyPropagatingEitherWay;
};

template<class T = MoveRelative>
class PyMoveRelative : public PyPropagatingEitherWay<T>
{
public:
	using PyPropagatingEitherWay<T>::PyPropagatingEitherWay;
};

template<class T = Connect>
class PyConnect : public PyConnecting<T>
{
public:
	using PyConnecting<T>::PyConnecting;
};

template<class T = SimpleGraspBase>
class PySimpleGraspBase : public PySerialContainer<T>
{
public:
	using PySerialContainer<T>::PySerialContainer;
};

template<class T = PickPlaceBase>
class PyPickPlaceBase : public PySerialContainer<T>
{
public:
	using PySerialContainer<T>::PySerialContainer;
};

template<class T = SimpleGrasp>
class PySimpleGrasp : public PySimpleGraspBase<T>
{
public:
	using PySimpleGraspBase<T>::PySimpleGraspBase;
};

template<class T = SimpleUnGrasp>
class PySimpleUnGrasp : public PySimpleGraspBase<T>
{
public:
	using PySimpleGraspBase<T>::PySimpleGraspBase;
};

template<class T = Pick>
class PyPick : public PyPickPlaceBase<T>
{
public:
	using PyPickPlaceBase<T>::PyPickPlaceBase;
};

template<class T = Place>
class PyPlace : public PyPickPlaceBase<T>
{
public:
	using PyPickPlaceBase<T>::PyPickPlaceBase;
};

template<class T = PredicateFilter>
class PyPredicateFilter : public PyWrapperBase<T>
{
public:
	using PyWrapperBase<T>::PyWrapperBase;
};

template<class T = ComputeIK>
class PyComputeIK : public PyWrapperBase<T>
{
public:
	using PyWrapperBase<T>::PyWrapperBase;
};

template<class T = PassThrough>
class PyPassThrough : public PyWrapperBase<T>
{
public:
	using PyWrapperBase<T>::PyWrapperBase;
};

}  // namespace stages
}  // namespace task_constructor
}  // namespace moveit