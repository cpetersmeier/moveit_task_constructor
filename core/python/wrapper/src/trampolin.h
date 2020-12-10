#include <moveit/task_constructor/stage.h>

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

template <class T = ComputeBase>
class PyComputeBase : public PyStage<T>
{
public:
	using PyStage<T>::PyStage;
};

template <class T = Generator>
class PyGenerator : public PyComputeBase<T>
{
public:
	using PyComputeBase<T>::PyComputeBase;

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

}  // namespace task_constructor
}  // namespace moveit