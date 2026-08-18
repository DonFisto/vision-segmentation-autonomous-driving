cmake_minimum_required(VERSION 3.10)
project(autonomy_interfaces)

# RoboStack's Python helper still uses the legacy FindPython modules. Keep
# that compatibility mode explicitly so newer CMake releases do not emit a
# policy warning or silently ignore the Conda interpreter.
if(POLICY CMP0148)
  cmake_policy(SET CMP0148 OLD)
endif()

# rosidl_generator_py needs the interpreter, development headers, libpython,
# and NumPy headers. In this project they all live inside the active Conda
# environment. Populate the variables before rosidl is discovered so a clean
# build does not depend on a previous CMake cache or long command-line flags.
if(DEFINED ENV{CONDA_PREFIX})
  set(_CONDA_PREFIX "$ENV{CONDA_PREFIX}")
  set(_CONDA_PYTHON "${_CONDA_PREFIX}/bin/python")

  if(EXISTS "${_CONDA_PYTHON}")
    execute_process(
      COMMAND "${_CONDA_PYTHON}" -c
        "import sys; print(f'{sys.version_info.major}.{sys.version_info.minor}')"
      OUTPUT_VARIABLE _CONDA_PYTHON_VERSION
      OUTPUT_STRIP_TRAILING_WHITESPACE
      RESULT_VARIABLE _CONDA_PYTHON_VERSION_RESULT
    )
    execute_process(
      COMMAND "${_CONDA_PYTHON}" -c
        "import sysconfig; print(sysconfig.get_path('include'))"
      OUTPUT_VARIABLE _CONDA_PYTHON_INCLUDE
      OUTPUT_STRIP_TRAILING_WHITESPACE
      RESULT_VARIABLE _CONDA_PYTHON_INCLUDE_RESULT
    )
    execute_process(
      COMMAND "${_CONDA_PYTHON}" -c
        "import numpy; print(numpy.get_include())"
      OUTPUT_VARIABLE _CONDA_NUMPY_INCLUDE
      OUTPUT_STRIP_TRAILING_WHITESPACE
      RESULT_VARIABLE _CONDA_NUMPY_INCLUDE_RESULT
    )

    if(_CONDA_PYTHON_VERSION_RESULT EQUAL 0)
      find_library(
        _CONDA_PYTHON_LIBRARY
        NAMES "python${_CONDA_PYTHON_VERSION}"
        PATHS "${_CONDA_PREFIX}/lib"
        NO_DEFAULT_PATH
      )
    endif()

    if(
      _CONDA_PYTHON_VERSION_RESULT EQUAL 0
      AND _CONDA_PYTHON_INCLUDE_RESULT EQUAL 0
      AND _CONDA_NUMPY_INCLUDE_RESULT EQUAL 0
      AND EXISTS "${_CONDA_PYTHON_INCLUDE}/Python.h"
      AND EXISTS "${_CONDA_NUMPY_INCLUDE}/numpy/arrayobject.h"
      AND _CONDA_PYTHON_LIBRARY
    )
      set(Python_ROOT_DIR "${_CONDA_PREFIX}" CACHE PATH "" FORCE)
      set(Python_FIND_VIRTUALENV ONLY CACHE STRING "" FORCE)
      set(Python_EXECUTABLE "${_CONDA_PYTHON}" CACHE FILEPATH "" FORCE)
      set(Python_INCLUDE_DIR "${_CONDA_PYTHON_INCLUDE}" CACHE PATH "" FORCE)
      set(Python_INCLUDE_DIRS "${_CONDA_PYTHON_INCLUDE}" CACHE PATH "" FORCE)
      set(Python_LIBRARY "${_CONDA_PYTHON_LIBRARY}" CACHE FILEPATH "" FORCE)
      set(Python_LIBRARIES "${_CONDA_PYTHON_LIBRARY}" CACHE FILEPATH "" FORCE)
      set(Python_NumPy_INCLUDE_DIR "${_CONDA_NUMPY_INCLUDE}" CACHE PATH "" FORCE)
      set(Python_NumPy_INCLUDE_DIRS "${_CONDA_NUMPY_INCLUDE}" CACHE PATH "" FORCE)
    endif()
  endif()
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(builtin_interfaces REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(rosidl_default_generators REQUIRED)
find_package(std_msgs REQUIRED)

rosidl_generate_interfaces(${PROJECT_NAME}
  "msg/LaneBoundary.msg"
  "msg/LaneSample.msg"
  "msg/LaneSegment.msg"
  "msg/LaneMap.msg"
  "msg/RoutePlan.msg"
  "msg/TrajectoryPoint.msg"
  "msg/Trajectory.msg"
  DEPENDENCIES
    builtin_interfaces
    geometry_msgs
    nav_msgs
    std_msgs
)

ament_export_dependencies(rosidl_default_runtime)
ament_package()
