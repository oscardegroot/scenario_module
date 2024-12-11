cmake_minimum_required(VERSION 3.8)
project(scenario_module)

add_compile_options(-std=c++17)

find_package(OpenMP)

if(OPENMP_FOUND)
  set(CMAKE_C_FLAGS "${CMAKE_C_FLAGS} ${OpenMP_C_FLAGS}")
  set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} ${OpenMP_CXX_FLAGS}")
  set(CMAKE_EXE_LINKER_FLAGS "${CMAKE_EXE_LINKER_FLAGS} ${OpenMP_EXE_LINKER_FLAGS}")
  message(WARNING "Compiling with OPENMP")
endif()

set(DEPENDENCIES
  roscpp
  std_msgs
  ros_tools
  mpc_planner_msgs
  mpc_planner_types
  mpc_planner_solver
  mpc_planner_util
)

find_package(catkin REQUIRED COMPONENTS ${DEPENDENCIES})

find_package(Eigen3 REQUIRED)
add_definitions(${EIGEN_DEFINITIONS})

# generate_dynamic_reconfigure_options(cfg/reconfigure.cfg)
catkin_package(
  CATKIN_DEPENDS ${DEPENDENCIES}
  INCLUDE_DIRS include
  LIBRARIES ${PROJECT_NAME}
)

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
)

# Library
add_library(${PROJECT_NAME} SHARED
  src/scenario_module.cpp
  src/config.cpp
  src/sampler.cpp
  src/partition_sampler.cpp
  src/safety_certifier.cpp
  src/polygon_search.cpp
  src/safe_horizon.cpp
)
add_dependencies(${PROJECT_NAME} ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
target_link_libraries(${PROJECT_NAME} ${catkin_LIBRARIES})

add_definitions(-DMPC_PLANNER_ROS)

install(TARGETS ${PROJECT_NAME}
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_GLOBAL_BIN_DESTINATION}
)

install(DIRECTORY include/${PROJECT_NAME}/
  DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
  FILES_MATCHING PATTERN "*.h"
  PATTERN ".svn" EXCLUDE
)

install(DIRECTORY include/third_party/
  DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}/../third_party
  FILES_MATCHING PATTERN "*.h"
  PATTERN ".svn" EXCLUDE
)
