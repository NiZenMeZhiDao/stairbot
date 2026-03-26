cmake_minimum_required(VERSION 3.8)
project(ares_usb)

#------------------------------------------------------------
# 依赖
#------------------------------------------------------------
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(builtin_interfaces REQUIRED)
find_package(rosidl_default_generators REQUIRED)
find_package(PkgConfig REQUIRED)
pkg_check_modules(LIBUSB REQUIRED libusb-1.0)

#------------------------------------------------------------
# 消息定义
#------------------------------------------------------------
# rosidl_generate_interfaces(${PROJECT_NAME}
#   "msg/MotorCmd.msg"
#   "msg/MotorState.msg"
#   DEPENDENCIES builtin_interfaces
# )

#------------------------------------------------------------
# 包含目录
#------------------------------------------------------------
include_directories(
  include
  ${CMAKE_CURRENT_SOURCE_DIR}/ARES_bulk_library
)

#------------------------------------------------------------
# 底层协议库 (静态)
#------------------------------------------------------------
add_library(ares_bulk STATIC
  ${CMAKE_CURRENT_SOURCE_DIR}/ARES_bulk_library/ares_protocol.cpp
)

target_include_directories(ares_bulk PUBLIC
  ${CMAKE_CURRENT_SOURCE_DIR}/ARES_bulk_library
  ${LIBUSB_INCLUDE_DIRS}
)

target_link_libraries(ares_bulk ${LIBUSB_LIBRARIES} pthread)

#------------------------------------------------------------
# 主节点可执行
#------------------------------------------------------------
add_executable(usb_bridge_node src/usb_bridge_node.cpp)
ament_target_dependencies(usb_bridge_node rclcpp sensor_msgs geometry_msgs std_msgs)
add_executable(test_publisher_node src/test_publisher_node.cpp)
ament_target_dependencies(test_publisher_node rclcpp sensor_msgs geometry_msgs std_msgs)

# 使用新的接口链接消息类型支持库
# rosidl_get_typesupport_target(cpp_typesupport_target "${PROJECT_NAME}" "rosidl_typesupport_cpp")
# target_link_libraries(usb_bridge_node ${cpp_typesupport_target})

target_link_libraries(usb_bridge_node ares_bulk)

#------------------------------------------------------------
# 安装
#------------------------------------------------------------
# install(TARGETS
#   usb_bridge_node
#   test_publisher_node
#   RUNTIME DESTINATION bin
#   ARCHIVE DESTINATION lib
#   LIBRARY DESTINATION lib
# )

install(TARGETS 
  usb_bridge_node 
  test_publisher_node
  DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY launch
  DESTINATION share/${PROJECT_NAME})

install(DIRECTORY include/
  DESTINATION include)
# 安装 bulk 协议头文件目录，便于其它包复用
install(DIRECTORY ARES_bulk_library/
  DESTINATION include/ARES_bulk_library
  FILES_MATCHING PATTERN "*.hpp")

# ament_export_include_directories(include)
# ament_export_include_directories(include/ARES_bulk_library)
# ament_export_dependencies(rosidl_default_runtime)
ament_package()