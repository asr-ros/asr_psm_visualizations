cmake_minimum_required(VERSION 2.8.3)
project(asr_psm_visualizations)

# C++11 is needed
add_definitions(-Wall -std=gnu++0x -ggdb)
set(CMAKE_BUILD_TYPE Debug)
include(CheckCXXCompilerFlag)
CHECK_CXX_COMPILER_FLAG("-std=c++11" COMPILER_SUPPORTS_CXX11)
CHECK_CXX_COMPILER_FLAG("-std=c++0x" COMPILER_SUPPORTS_CXX0X)
if(COMPILER_SUPPORTS_CXX11)
    set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++11")
elseif(COMPILER_SUPPORTS_CXX0X)
    set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++0x")
else()
        message(STATUS "The compiler ${CMAKE_CXX_COMPILER} has no C++11 support. Please use a different C++ compiler.")
endif()
find_package(OpenCV 2 REQUIRED)

find_package(catkin REQUIRED COMPONENTS
        roscpp
        tf
        asr_resources_for_psm
        std_msgs
)
find_package(cmake_modules)
find_package(Eigen REQUIRED)

find_package(Boost REQUIRED COMPONENTS
        iostreams
        system
        filesystem
)



catkin_package(
        INCLUDE_DIRS include
        LIBRARIES visualization_server_lib
        CATKIN_DEPENDS roscpp tf asr_resources_for_psm std_msgs
        DEPENDS Eigen Boost
)

include_directories(
        include
        ${OpenCV_INCLUDE_DIRS}
        ${Eigen_INCLUDE_DIRS}
        ${catkin_INCLUDE_DIRS}
        ${Boost_INCLUDE_DIRS}
)



add_library(visualization_server_lib
                      src/visualization/EnvironmentVisualization.cpp
                      src/visualization/GaussianVisualization.cpp
                      src/visualization/psm/ProbabilisticSceneModelVisualization.cpp
                      src/visualization/psm/ProbabilisticSceneVisualization.cpp
                      src/visualization/psm/ProbabilisticPrimarySceneObjectVisualization.cpp
                      src/visualization/psm/ProbabilisticSecondarySceneObjectVisualization.cpp
                      src/visualization/psm/helper/KinematicChainVisualizer.cpp
                      src/visualization/psm/helper/GaussianKernelVisualizer.cpp
                      src/visualization/psm/helper/SampleVisualizer.cpp
                      src/visualization/psm/helper/CoordinateFrameVisualizer.cpp
                      src/visualization/psm/helper/AbstractExtendedVisualizer.cpp
                      src/visualization/psm/helper/AbstractVisualizer.cpp
                      src/visualization/gnuplot/GnuplotVisualization.cpp
                      src/visualization/gnuplot/GMMGnuplotVisualization.cpp
                      src/helper/ColorHelper.cpp
)

target_link_libraries(visualization_server_lib
        ${OpenCV_LIBRARIES}
        ${Boost_LIBRARIES}
        ${catkin_LIBRARIES}
        ${Eigen_LIBRARIES}
)





