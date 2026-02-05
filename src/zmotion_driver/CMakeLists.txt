cmake_minimum_required(VERSION 3.8)
project(zmotion_driver)

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
find_package(rclcpp_action REQUIRED)
find_package(std_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(motion_msgs REQUIRED)

# 更明确地指定库路径，使用绝对路径或项目相关路径
set(ZMOTION_LIB_DIR ${CMAKE_CURRENT_SOURCE_DIR}/lib)
link_directories(${ZMOTION_LIB_DIR})

# 或者使用find_library命令查找库
find_library(ZMOTION_LIBRARY
  NAMES zmotion
  PATHS ${ZMOTION_LIB_DIR}
  NO_DEFAULT_PATH
)

# 设置运行时库路径
set(CMAKE_BUILD_WITH_INSTALL_RPATH TRUE)
set(CMAKE_INSTALL_RPATH "$ORIGIN/../lib/${PROJECT_NAME}")

add_executable(zmotion_node
    src/main.cpp
    src/zmcaux.cpp
    src/zmc_controller.cpp
)

# 设置目标的包含目录
target_include_directories(zmotion_node PUBLIC
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
    $<INSTALL_INTERFACE:include>
)

ament_target_dependencies(zmotion_node rclcpp rclcpp_action std_msgs sensor_msgs motion_msgs)

target_link_libraries(zmotion_node
    # ${ZMOTION_LIBRARY}
    pthread
    ${CMAKE_CURRENT_SOURCE_DIR}/lib/libzmotion.so
)

# 设置rpath，确保运行时能够找到库文件
set_target_properties(zmotion_node PROPERTIES
  INSTALL_RPATH "$ORIGIN"
  BUILD_WITH_INSTALL_RPATH TRUE
)

add_executable(zmotion_test
    test/test.cpp
    src/zmcaux.cpp
)

set_target_properties(zmotion_test PROPERTIES
  INSTALL_RPATH "$ORIGIN"
  BUILD_WITH_INSTALL_RPATH TRUE
)

target_include_directories(zmotion_test PUBLIC
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
    $<INSTALL_INTERFACE:include>
)

target_link_libraries(zmotion_test 
    ${ZMOTION_LIBRARY}
    pthread
    ${CMAKE_CURRENT_SOURCE_DIR}/lib/libzmotion.so
)

# 设置运行时库搜索路径
# if(UNIX AND NOT APPLE)
#     set_target_properties(zmotion_test PROPERTIES
#         INSTALL_RPATH "$ORIGIN/../lib/${PROJECT_NAME}"
#         BUILD_WITH_INSTALL_RPATH TRUE
#     )
# endif()

install(TARGETS 
  zmotion_node
  zmotion_test
  DESTINATION lib/${PROJECT_NAME}
)

# 安装libzmotion.so库文件
install(FILES ${ZMOTION_LIB_DIR}/libzmotion.so
  DESTINATION lib/${PROJECT_NAME}
)

# 安装头文件
install(DIRECTORY include/${PROJECT_NAME}/
  DESTINATION include/${PROJECT_NAME}/
  FILES_MATCHING PATTERN "*.h"
  PATTERN "readme.txt" EXCLUDE
)

ament_package()