cmake_minimum_required(VERSION 2.8.3)
project(canny_devo)

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

cs_add_executable(canny_devo_node
    src/canny_devo_node.cpp
)

#target_link_libraries(esvo_plus_offline_slam ${OpenCV_LIBS})
# make the executable
#cs_add_executable(esvo_plus_node_slam
#    src/esvo_plus_node_slam.cpp
#)

# link the executable to the necesarry libs
target_link_libraries(canny_devo_node
   canny_devo_offline
   esvo_core_LIB
)

# link the executable to the necesarry libs
#target_link_libraries(esvo_plus_node_slam
#   esvo_plus_offline_slam
#)

cs_install()
