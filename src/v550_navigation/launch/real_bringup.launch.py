import os
from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    IncludeLaunchDescription,
    SetEnvironmentVariable,
)
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

''' 本launch文件的作用
只负责实车基础环境：
1. 底盘、EKF、URDF、IMU
2. 雷达
3. /scan -> /scan_slam
4. slam_toolbox 在线建图
'''

def generate_launch_description():
    """
    V550ROS小车实车运行ego_planner的基础环境启动文件
    只负责实车基础环境：
    1. 底盘、EKF、URDF、IMU
    2. 雷达
    3. /scan -> /scan_slam
    4. slam_toolbox 在线建图
    """
    
    # v550_navigation 包路径
    pkg = get_package_share_directory('v550_navigation')
    # 实车底盘包路径
    base_pkg = get_package_share_directory('turn_on_wheeltec_robot')
    # launch 参数
    bringup_hardware = LaunchConfiguration('bringup_hardware')
    start_slam = LaunchConfiguration('start_slam')
    # SLAM 参数文件
    slam_params = os.path.join(
        pkg,
        'config',
        'slam_v550.yaml',
    )

    # 真实底盘、EKF、URDF、IMU
    hardware = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(
                base_pkg,
                'launch',
                'turn_on_wheeltec_robot.launch.py'
            )
        ),
        condition = IfCondition(bringup_hardware),
    )
    # 真实 M10_P 雷达
    lidar = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(
                base_pkg,
                'launch',
                'wheeltec_lidar.launch.py'
            )
        ),
        condition = IfCondition(bringup_hardware),
    )


    # 雷达归一化
    # /scan -> /scan_slam
    # slam_toolbox 使用固定点数的扫描
    normalizer = Node(
        package='v550_navigation',
        executable='normalize_scan',
        name='v550_scan_normalizer',
        parameters=[
            {
                 # 实车雷达原始话题
                'input_topic':'/scan',
                # slam_toolbox 使用的话题
                'output_topic':'scan_slam',
                 # 固定输出点数
                'scan_points':800,
                #前方大约200视角
                'fov_min':-1.7453292519943295,
                'fov_max':1.7453292519943295,
                #实车不使用仿真时间
                'use_sim_time':False,
            }
        ],
        condition = IfCondition(start_slam),
        output='screen',
    )

    slam = Node(
        package='slam_toolbox',
        executable='async_slam_toolbox_node',
        name='slam_toolbox',
        parameters=[
            # 公共 SLAM 参数
            slam_params,

            # 实车覆盖参数
            {
                'scan_topic':'/scan_slam',

                'use_sim_time':False,
            },
        ],
        condition = IfCondition(start_slam),
        output='screen',
    )

    return LaunchDescription([
         # 是否启动真实底盘、EKF、雷达
        DeclareLaunchArgument(
            'bringup_hardware',
            default_value='true',
            description='启动实车底盘 EKF 雷达' 
        ),
        # 是否启动 SLAM
        DeclareLaunchArgument(
            'start_slam',
            default_value='ture',
            description='启动 slam_toolbox 在线建图',
        ),
        # ROS 通信隔离。
        # 如果 RViz 在另一台电脑上运行，需要改成 0。
        SetEnvironmentVariable( 
            'ROS_LOCATION_ONLY',
            '1',
        ),
        # 与仿真保持一致，使用 FastRTPS。
        # 如果车机系统统一使用 CycloneDDS，可以删除或改掉。
        SetEnvironmentVariable(
            'RMW_IMPLEMENTATION',
            'rmw_fastrtps_cpp',
        ),

        hardware,
        lidar,
        normalizer,
        slam,

    ])

