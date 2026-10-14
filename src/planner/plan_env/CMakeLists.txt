cmake_minimum_required(VERSION 3.8)
project(plan_env)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_BUILD_TYPE "Release")

add_compile_options(-Wall -O3 -g)

# 查找依赖
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)
find_package(Eigen3 REQUIRED)

include_directories(
  include
  ${EIGEN3_INCLUDE_DIRS}
)

#对外导出头文件
ament_export_include_directories(include)

# 让依赖 plan_env 的包自动获得这些依赖的 include path
ament_export_dependencies(
    rclcpp
    nav_msgs
    sensor_msgs
    geometry_msgs
    tf2
    tf2_ros
    tf2_geometry_msgs
    Eigen3
)
# 编译二维地图库
add_library(plan_env SHARED
    src/grid_map.cpp
)

# 链接依赖
ament_target_dependencies(plan_env
    rclcpp
    nav_msgs
    sensor_msgs
    geometry_msgs
    tf2
    tf2_ros
    tf2_geometry_msgs
    Eigen3
)

# 安装库
install(TARGETS plan_env
    DESTINATION lib
)

#安装头文件
install(DIRECTORY include/${PROJECT_NAME}/
    DESTINATION include/${PROJECT_NAME}
)

ament_export_libraries(plan_env)

ament_package()
