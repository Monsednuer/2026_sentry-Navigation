cmake_minimum_required(VERSION 3.5)
project(mpc_follower)

# Default to C99
if(NOT CMAKE_C_STANDARD)
  set(CMAKE_C_STANDARD 99)
endif()

# Default to C++14
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 14)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(visualization_msgs REQUIRED)
find_package(Eigen3 REQUIRED)
find_package(visualization_msgs REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)
find_package(PkgConfig REQUIRED)
find_package(angles REQUIRED)
find_package(pcl_conversions REQUIRED)
# pkg_check_modules(NLOPT REQUIRED nlopt) 

add_executable(mpc_node src/mpc_node.cpp src/mpc_controller.cpp)
# add_executable(benchmark_mpc src/benchmark_mpc.cpp)

include_directories(
  include
  ${EIGEN3_INCLUDE_DIRS}
  ${NLOPT_INCLUDE_DIRS}
)
# find_path(NLOPT_INCLUDE_DIR nlopt.hpp PATHS /usr/include /usr/local/include)
# find_library(NLOPT_LIBRARY nlopt PATHS /usr/lib /usr/local/lib)
find_path(NLOPT_INCLUDE_DIR NAMES nlopt.hpp)
find_library(NLOPT_LIBRARY NAMES nlopt)

if(NOT NLOPT_INCLUDE_DIR OR NOT NLOPT_LIBRARY)
  message(FATAL_ERROR "nlopt not found")
endif()

target_include_directories(mpc_node PRIVATE ${NLOPT_INCLUDE_DIR})
target_link_libraries(mpc_node ${NLOPT_LIBRARY})
# target_link_libraries(benchmark_mpc ${PROJECT_NAME})


ament_target_dependencies(mpc_node 
  rclcpp 
  std_msgs 
  nav_msgs 
  geometry_msgs 
  sensor_msgs
  visualization_msgs
  tf2
  tf2_ros
  tf2_geometry_msgs
  angles
  pcl_conversions
)

install(TARGETS
  mpc_node
  DESTINATION lib/${PROJECT_NAME})

install(DIRECTORY include/
  DESTINATION include/)

install(DIRECTORY cfg/
  DESTINATION share/${PROJECT_NAME}/cfg/)

install(DIRECTORY launch/
  DESTINATION share/${PROJECT_NAME}/launch/)


if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  # the following line skips the linter which checks for copyrights
  # uncomment the line when a copyright and license is not present in all source files
  #set(ament_cmake_copyright_FOUND TRUE)
  # the following line skips cpplint (only works in a git repo)
  # uncomment the line when this package is not in a git repo
  #set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
