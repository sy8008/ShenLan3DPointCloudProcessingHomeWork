cmake_minimum_required(VERSION 3.0.0)
project(pca_node_proj VERSION 0.1.0)
SET(CMAKE_CXX_FLAGS -std=c++11)
set(CMAKE_BUILD_TYPE Debug)
FIND_PACKAGE( PCL REQUIRED COMPONENTS common io filters visualization features)
find_package (Eigen3 3.3 REQUIRED NO_MODULE)
ADD_DEFINITIONS( ${PCL_DEFINITIONS} )

include_directories(${PCL_INCLUDE_DIRS} ${Boost_INCLUDE_DIRS} ${Eigen_INCLUDE_DIRS})
add_executable(normal_estimater src/normal_estimate.cpp)

target_link_libraries(normal_estimater ${PCL_LIBRARIES} ${Boost_LIBRARIES} Eigen3::Eigen)
