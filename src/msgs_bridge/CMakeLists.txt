cmake_minimum_required(VERSION 3.8)
project(msgs_bridge)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(motion_msgs REQUIRED)
find_package(slms_interface REQUIRED)
find_package(rclcpp_action REQUIRED)


# 添加可执行文件
add_executable(status_converter src/status_converter.cpp)
ament_target_dependencies(status_converter 
    rclcpp 
    std_msgs 
    motion_msgs 
    slms_interface
  )

add_executable(action_converter src/action_converter.cpp)
ament_target_dependencies(action_converter 
    rclcpp 
    rclcpp_action
    motion_msgs
    slms_interface
  )

# 安装路径
install(TARGETS
  status_converter
  action_converter
  DESTINATION lib/${PROJECT_NAME}
)

# 安装 launch 文件夹到 share 目录
install(DIRECTORY launch
  DESTINATION share/${PROJECT_NAME}
)

ament_package()