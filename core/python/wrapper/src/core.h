#include <moveit/task_constructor/stage.h>
#include <moveit/task_constructor/container.h>
#include <moveit/task_constructor/task.h>
#include <pybind11/pybind11.h>

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

template <class T = PropagatingEitherWay>
class PyPropagatingEitherWay : public PyStage<T>
{
public:
	using PyStage<T>::PyStage;
};

template <class T = PropagatingBackward>
class PyPropagatingBackward : public PyPropagatingEitherWay<T>
{
public:
	using PyPropagatingEitherWay<T>::PyPropagatingEitherWay;
};

template <class T = PropagatingForward>
class PyPropagatingForward : public PyPropagatingEitherWay<T>
{
public:
	using PyPropagatingEitherWay<T>::PyPropagatingEitherWay;
};

template <class T = Connecting>
class PyConnecting : public PyStage<T>
{
public:
	using PyStage<T>::PyStage;
	void compute(const InterfaceState& from, const InterfaceState& to) override {
		PYBIND11_OVERRIDE_PURE(void, T, compute, from, to);
	}
};

template <class T = ContainerBase>
class PyContainerBase : public PyStage<T>
{
public:
	using PyStage<T>::PyStage;

	bool insert(Stage::pointer&& stage, int before = -1) override { PYBIND11_OVERRIDE(bool, T, insert, stage, before); }

	Stage::pointer remove(int pos) override { PYBIND11_OVERRIDE(Stage::pointer, T, remove, pos); }

	Stage::pointer remove(Stage* child) override { PYBIND11_OVERRIDE(Stage::pointer, T, remove, child); }

	void clear() override { PYBIND11_OVERRIDE(void, T, clear, ); }

	bool canCompute() const override { PYBIND11_OVERRIDE_PURE(bool, T, canCompute, ); }

	void compute() override { PYBIND11_OVERRIDE_PURE(void, T, compute, ); }

	void onNewSolution(const SolutionBase& s) override { PYBIND11_OVERRIDE_PURE(void, T, onNewSolution, s); }
};

template <class T = SerialContainer>
class PySerialContainer : public PyContainerBase<T>
{
public:
	using PyContainerBase<T>::PyContainerBase;
};

template <class T = ParallelContainerBase>
class PyParallelContainerBase : public PyContainerBase<T>
{
public:
	using PyContainerBase<T>::PyContainerBase;
};

template <class T = WrapperBase>
class PyWrapperBase : public PyParallelContainerBase<T>
{
public:
	using PyParallelContainerBase<T>::PyParallelContainerBase;
};

template <class T = Alternatives>
class PyAlternative : public PyParallelContainerBase<T>
{
public:
	using PyParallelContainerBase<T>::PyParallelContainerBase;
};

template <class T = Fallbacks>
class PyFallbacks : public PyParallelContainerBase<T>
{
public:
	using PyParallelContainerBase<T>::PyParallelContainerBase;
};

template <class T = Merger>
class PyMerger : public PyParallelContainerBase<T>
{
public:
	using PyParallelContainerBase<T>::PyParallelContainerBase;
};

template <class T = Task>
class PyTask : public PyWrapperBase<T>
{
public:
	using PyWrapperBase<T>::PyWrapperBase;
};

}  // namespace task_constructor
}  // namespace moveit