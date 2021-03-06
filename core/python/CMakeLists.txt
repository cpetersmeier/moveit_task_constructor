# As we rely on some not yet released pybind11 PRs, we are employing
# our own fork of it, imported via git submodule.
# See https://github.com/pybind/pybind11/pull/2687.

# pybind11 must use the ROS python version
set(PYBIND11_PYTHON_VERSION ${PYTHON_VERSION_STRING})

# create symlink to grant access to downstream packages in devel space
add_custom_target(pybind11_devel_symlink ALL COMMAND ${CMAKE_COMMAND} -E create_symlink
	${CMAKE_CURRENT_SOURCE_DIR}/pybind11
	${CATKIN_DEVEL_PREFIX}/share/${PROJECT_NAME}/pybind11)

# configure pybind11 install for use by downstream packages in install space
set(PYBIND11_INSTALL ON CACHE INTERNAL "Install pybind11")
set(CMAKE_INSTALL_INCLUDEDIR include/moveit/python)
set(PYBIND11_CMAKECONFIG_INSTALL_DIR ${CATKIN_PACKAGE_SHARE_DESTINATION}/cmake
    CACHE INTERNAL "install path for pybind11 cmake files")

# source pybind11 folder, which exposes its targets and installs it
add_subdirectory(pybind11)


# C++ wrapper code
add_subdirectory(wrapper)

if(CATKIN_ENABLE_TESTING)
	find_package(rostest REQUIRED)

	# Add folders to be run by python nosetests
	catkin_add_nosetests(test)

	# Add rostests
	add_rostest(test/rostest_mtc.test)
endif()

roslint_python()
