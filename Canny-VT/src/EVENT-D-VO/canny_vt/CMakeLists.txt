cmake_minimum_required(VERSION 2.8.3)
project(canny_vt)

# To be consistent with the configuration in esvo_core
set(CMAKE_CXX_STANDARD 17)
# SET(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++11 -O3")
set(CMAKE_CXX_FLAGS "-O3 -fopenmp")

find_package(catkin_simple REQUIRED)
catkin_simple(ALL_DEPS_REQUIRED)

# SET(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++11 -O3")

find_package(OpenCV 3 REQUIRED)

include_directories(include ${catkin_INCLUDE_DIRS} ${OpenCV_INCLUDE_DIRS})
link_directories(${PROJECT_SOURCE_DIR}/lib)

# make the executable
cs_add_executable(canny_vt_node_slam
    src/canny_vt_node_slam.cpp
)

# link the executable to the necesarry libs
target_link_libraries(canny_vt_node_slam
   canny_vt_offline_slam
   esvo_core_LIB
)

cs_install()
