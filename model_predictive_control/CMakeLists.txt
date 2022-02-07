cmake_minimum_required(VERSION 3.10)
project(model_predictive_control)

include(CheckCXXCompilerFlag)
CHECK_CXX_COMPILER_FLAG("-std=c++11" COMPILER_SUPPORTS_CXX11)
CHECK_CXX_COMPILER_FLAG("-std=c++0x" COMPILER_SUPPORTS_CXX0X)

if (COMPILER_SUPPORTS_CXX11)
    set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++11")
elseif (COMPILER_SUPPORTS_CXX0X)
    set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++0x")
else ()
    message(FATAL_ERROR "The compiler ${CMAKE_CXX_FLAGS} doesn't have C++11 support.
                         Please use a different C++ compiler")
endif ()

set(ADDITIONAL_CXX_FLAG "-Wall -O3 -march=native")
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} ${ADDITIONAL_CXX_FLAG}")

find_package(Eigen3 REQUIRED)
find_package(PythonLibs 2.7 COMPONENTS Development NumPy)

include_directories(${EIGEN3_INCLUDE_DIRS})

add_executable(model_predictive_control main.cpp)
target_include_directories(model_predictive_control PRIVATE ${PYTHON_INCLUDE_DIRS})
target_link_libraries(model_predictive_control ${PYTHON_LIBRARIES})
