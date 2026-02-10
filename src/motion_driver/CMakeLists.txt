cmake_minimum_required(VERSION 3.8)
project(motion_driver)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# 1. 查找依赖
find_package(ament_cmake REQUIRED)
find_package(ament_cmake_python REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclpy REQUIRED)
find_package(rclcpp_action REQUIRED)
find_package(rosidl_default_generators REQUIRED)

# 2. 生成 Action 接口
rosidl_generate_interfaces(${PROJECT_NAME}
  "action/MoveToTarget.action"
)

# --- 2. 处理 motion_driver 文件夹 (作为模块供其他包 import) ---
# 注意：这里我们避开 ament_python_install_package 的命名冲突
# 手动将模块安装到 Python 的 site-packages 路径
# ament_cmake_python_install_package(${PROJECT_NAME})

# 替代 ament_cmake_python_install_package 的原始写法：
install(DIRECTORY motion_driver
  DESTINATION "${PYTHON_INSTALL_DIR}"
)

# 4. 安装 Python 脚本 (让 ros2 run 能执行)
set(PYTHON_TOOLS
  "action_server"
  "action_client"
)

foreach(tool ${PYTHON_TOOLS})
  install(PROGRAMS scripts/${tool}.py
    DESTINATION lib/${PROJECT_NAME}
    RENAME ${tool}
  )
endforeach()

ament_export_dependencies(rosidl_default_runtime)

# 5. 导出
ament_package()