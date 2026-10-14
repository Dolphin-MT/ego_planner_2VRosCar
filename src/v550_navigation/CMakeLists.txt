cmake_minimum_required(VERSION 3.8)
# 定义当前ROS包
project(v550_navigation)

# 包中没有C++编译目标，只需要ament_cmake
find_package(ament_cmake REQUIRED)


# 安装配置、启动、世界、模型和 RViz 配置
install(
  DIRECTORY config launch rviz urdf worlds
  DESTINATION share/${PROJECT_NAME}
)

# 安装 Python 可执行节点
# 安装后可以用 ros2 run v550_navigation normalize_scan 启动
install(
  PROGRAMS 
  scripts/normalize_scan.py
  # scripts/tf_to_odom_world.py
  DESTINATION lib/${PROJECT_NAME}
)

# install中将编译后的包名后缀去掉的用法
# 安装并重命名 tf_to_odom_world.py
install(
  PROGRAMS scripts/tf_to_odom_world.py
  DESTINATION lib/${PROJECT_NAME}
  RENAME tf_to_odom_world  # 安装后去掉 .py 后缀
)



ament_package()
