import os
# 用于查找 ROS 2 包的 share 目录
from ament_index_python.packages import get_package_share_directory

# ROS 2 launch 基础模块
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.actions import IncludeLaunchDescription
from launch.actions import TimerAction

from launch.launch_description_sources import PythonLaunchDescriptionSource

from launch.substitutions import LaunchConfiguration

from launch_ros.actions import Node

def generate_launch_description():
    """
    这个 launch 文件负责启动 V550 麦轮小车的 2D Ego 仿真系统。
    总体流程：
        Gazebo + V550_mec
            -> /scan
            -> /odom_combined
            -> TF
        slam_toolbox
            -> /map
            -> map -> odom_combined

        tf_to_odom_world.py
            -> /odom_world

        ego_planner
            -> /planning/bspline

        mecanum_traj_server
            -> /cmd_vel

        V550_mec
            -> 底盘运动
    """
    # 获取 v550_navigation 包的 share 目录
    v550_pkg = get_package_share_directory('v550_navigation')

    rviz_config = os.path.join(
        v550_pkg,
        'rviz',
        'v550_ego.rviz'
    )

    simulation = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(
                v550_pkg,
                'launch',
                'simulation_nav.launch.py'
            )
        ),
        launch_arguments={
            'use_sim_time' :'True',
            'use_rviz': 'false',
            'use_gui': 'True',#貌似有问题
            'start_slam': 'true',
            'start_nav2':'False', #这样在启动simulation文件时就不会启动nav2的导航文件
        }.items()

    )
    #启动 tf_to_odom_world.py
    tf_to_odom_world = Node(
        package='v550_navigation',
        executable='tf_to_odom_world',
        name='tf_to_odom_world',
        output='screen',
        parameters=[
            {'use_sim_time': True}
        ],
    )

    #启动Egp_planner
    ego_planner =Node(
        package='ego_planner',
        executable='ego_planner_node',
        name='ego_planner_node',
        output='screen',
        parameters=[
            {'use_sim_time':True},
            #FSM状态机
            {'fsm/flight_type':1},

            {'fsm/thresh_replan_time':1.0},
            {'fsm/thresh_no_replan_meter':0.5},
            {'fsm/planning_horizon':3.0},
            {'fsm/planning_horizen_time':3.0},
            {'fsm/emergency_time':1.0},
            {'fsm/realworld_experiment':False},
            {'fsm/fail_safe':True},
            {'fsm/waypoint_num':0},

            # planner manager参数
            {'manager/drone_id':-1},
            {'manager/max_vel':0.5},
            {'manager/max_acc':1.0},
            {'manager/max_jerk':2.0},
            {'manager/control_points_distance':0.4},
            {'manager/feasibility_tolerance':0.05},
            {'manager/planning_horizon':3.0},
            {'manager/use_distinctive_trajs':False},

            # B样条参数优化
            {'optimization/lambda_smooth':1.0},
            {'optimization/lambda_collision':0.5},
            {'optimization/lambda_feasibility':0.1},
            {'optimization/lambda_fitness':1.0},
            {'optimization/dist0':0.5},
            {'optimization/swarm_clearance':0.5},
            {'optimization/max_vel':0.5},
            {'optimization/max_acc':1.0},
            {'bspline/limit_vel':0.5},
            {'bspline/limit_acc':1.0},
            {'bspline/limit_ratio':1.1},

            # 2D plan_env 参数
            {'map_frame':'map'},
            {'map_topic':'/map'},
            {'scan_topic':'/scan'},
            {'inflate_radius':0.35},
            {'max_scan_range':6.0},
            #每隔4束雷达取一个点
            {'scan_step':4},

            {'fsm/goal_reach_thresh': 0.10},
            {'fsm/goal_reach_vel_thresh': 0.05},
        ],
        remappings=[
            # 如果以后你把目标话题改回 /move_base_simple/goal，
            # 可以使用下面这一行进行重映射：
            # ('/move_base_simple/goal', '/goal_pose'),
        ],
    )
    # 启动麦伦轨迹跟踪器
    mecanum_traj_server = Node(
        package='ego_planner',
        executable='mecanum_traj_server',
        name='mecanum_traj_server',
        output='screen',
        parameters=[
            {'use_sim_time':True},
            {'max_linear_velocity':0.5},
            {'max_angular_velocity':1.5},
            {'yaw_gain':1.5},
            {'position_kp_x': 1.0},
            {'position_kp_y': 1.0},
            {'goal_position_tolerance': 0.05},
        ]
    )

    #启动rviz
    rviz = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        arguments=['-d',rviz_config],
        parameters=[
            {'use_sim_time': True}
        ],
        output= 'screen',
    )
    #延时启动Ego和轨迹跟踪器
    delayed_planner = TimerAction(
        period=3.0,
        actions=[ego_planner],
    )
    delayed_tracker = TimerAction(
        period=3.0,
        actions=[mecanum_traj_server],
    )

    return LaunchDescription([
        # 启动Gazebo+v550+slam_toolbox
        simulation,

        # 发布/odom_word
        tf_to_odom_world,

        #延时启动Ego规划器
        delayed_planner,

        # 启动麦轮跟踪器
        delayed_tracker,

        # 启动Rviz
        rviz,])
        
    