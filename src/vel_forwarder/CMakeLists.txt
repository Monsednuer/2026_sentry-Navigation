# cmake_minimum_required(VERSION 3.8)
cmake_minimum_required(VERSION 3.10)
project(vel_forwarder)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(geometry_msgs REQUIRED)
# find_package(rosidl_default_generators REQUIRED)
find_package(std_msgs REQUIRED)
find_package(gimbal_driver REQUIRED)

# # 生成自定义消息接口
# rosidl_generate_interfaces(${PROJECT_NAME}
#   msg/Vel.msg
#   msg/BuffData.msg
#   msg/GameData.msg
#   msg/GimbalAngles.msg
#   msg/Health.msg
#   msg/PositionData.msg
#   msg/UWBPos.msg
#   DEPENDENCIES std_msgs
# )

# 创建可执行节点
add_executable(forwarder_node src/forwarder.cpp)

target_include_directories(forwarder_node PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)

ament_target_dependencies(forwarder_node
  rclcpp
  geometry_msgs
  gimbal_driver
)

# # 链接生成的消息库
# rosidl_get_typesupport_target(cpp_typesupport_target ${PROJECT_NAME} "rosidl_typesupport_cpp")
# target_link_libraries(forwarder_node "${cpp_typesupport_target}")

install(TARGETS forwarder_node
  DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY launch
  DESTINATION share/${PROJECT_NAME}
)


ament_package()