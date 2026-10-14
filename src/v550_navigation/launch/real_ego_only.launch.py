import os
from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch.actions import(
    DeclareLaunchArgument,
    IncludeLaunchDescription,
    TimerAction,
)
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

"""作用
1. include real_bringup.launch.py
2. 启动 tf_to_odom_world
3. 启动 ego_planner_node
4. 启动 mecanum_traj_server
5. 启动 Ego 专用 RViz

"""
def generate_launch_description():
    """
    v550 实车Ego only启动文件

    启动链路：
        real_bringup.launch.py
        ->底盘 雷达 /scan_slam slam_toolbox

        tf_to_odom_world
            ->map->base_footprint
            ->/odom_world
        ego_planner_node
            ->/planning/bspline
        mecanum_traj_server
            ->/cmd_vel
        V550 实车底盘
            ->运动
    """
     # v550_navigation 包路径
    pkg = get_package_share_directory('v550)navigation')
    # launch 参数
    bringup_hardware = LaunchConfiguration('bringup_hardware')
    start_slam = LaunchConfiguration('start_slam')
    use_rviz = LaunchConfiguration('use_rviz')
    # 实车基础环境：底盘 + 雷达 + SLAM
    real_bringup = IncludeLaunchDescription(

        PythonLaunchDescriptionSource(
            os.path.join(
                pkg,
                'launch',
                'real_bringup.launch.py',
            )
        ),
        launch_arguments={
            # 默认由本 launch 启动底盘和雷达
            'bringup_hardware':bringup_hardware,
            # 默认启动 slam_toolbox
            'start_slam':start_slam,
        }.items(),
    )

    # 把 map -> base_footprint 转成 /odom_world
    # Ego 的 odometryCallback 订阅 odom_world
    tf_to_odom_world = Node(
        package='v550_navigation',
        executable='tf_to_odom_world',
        name = 'tf_to_odom_world',
        output='screen',
        parameters=[
            {
                # 实车使用系统时间
                'use_sim_time': False
            },
        ],
    )
     # Ego planner
    ego_planner = Node(
        package='ego_planner',
        executable='ego_planner_node',
        name = 'ego_planner_node',
        output='screen',
        parameters=[
            {
                # 实车不使用仿真时间
                'use_sim_time':False,
                 # FSM 状态机
                'fsm/flight_type':1,
                'fsm/thresh_replan_time':1.0,
                'fsm/thresh_no_replan_meter':0.3,
                'fsm/planning_horizon':3.0,
                'fsm/planning_horizon_time':3.0,
                'fsm/emergency_time':1.0,
                'fsm/realworld_experiment':False,
                'fsm/fail_safe':True,

                #planner manager
                'manager/drone_id':-1,
                'manager/max_vel':.3,
                'manager/max_acc':0.5,
                'manager/max_jerk': 1.0,
                'manager/control_points_distance': 0.3,
                'manager/feasibility_tolerance': 0.05,
                'manager/planning_horizon': 2.0,
                'manager/use_distinctive_trajs': False,

                #B样条优化参数
                'optimization/lambda_smooth':1.0,
                'optimization/lambda_collision':0.5,
                'optimization/lambda_feasibility':0.1,
                'optimization/lambda_fitness':1.0,
                'optimization/dist0':0.4,
                'optimization/max_vel':0.3,
                'optimization/max_acc':0.5,

                #B样条限制
                'bspline/limit_vel':0.3,
                'bspline/limit_acc':0.5,
                'bspline/limit_ratio':1.1,

                #2DGridmap
                'map_frame':'map',
                'map_topic':'/map',

                # GridMap 使用原始雷达更新障碍物
                'scan_topic':'/scan',

                # 实车膨胀半径，先按车体尺寸和安全余量调
                'inflate_radius':0.35,

                # 最大使用雷达距离
                'max_scan_range':6.0,

                 #每隔 4 束雷达取一个点，降低计算量
                'scan_step': 4
            }
        ],
    )

     # V550 麦轮轨迹跟踪器
    mecanum_server = Node(
        package='ego_planner',
        executable='mecanum_traj_server',
        name='mecanum_traj_server',
        output='screen',
        parameters=[
            {
                # 实车不使用仿真时间
                'use_sim_time':False,
                # 实车先低速
                'max_linear_velocity':0.3,
                'max_angular_velocity':1.0,
                'yaw_gain':1.0,
                # 位置闭环
                'position_kp_x':1.0,
                'position_kp_y':1.0,
                'goal_position_tolerance':0.05,
            }
        ],
     )
    #EGO专用rviz
    rviz = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        arguments=[
            '-d',
            os.path.join(
                pkg,
                'rviz',
                'v550_ego.rviz',
            ),
        ],
        parameters=[
            {
                'use_sim_time':False,
            }
        ],
        condition = IfCondition(use_rviz),
        output='screen',
    )

    #ego和跟踪器延迟启动，给底盘 雷达 slam留出时间
    delayed_planner = TimerAction(
        period=3.0,
        actions=[ego_planner],
    )
    delayed_tracker = TimerAction(
        period=3.0,
        actions=[mecanum_server],
    )

    return LaunchConfiguration([
        # 是否启动真实底盘、EKF、雷达
        DeclareLaunchArgument(
            'bringup_hardware',
            default_value='true',
            description='启动真实底盘 EKF 雷达',
        ),
         # 是否启动 slam_toolbox
        DeclareLaunchArgument(
            'start_slam',
            default_value='true',
            description='启动slam_toolbox',
        ),
        # 是否启动 RViz
        DeclareLaunchArgument(
            'use_rviz',
            default_value='true',
            description='启动EGO专用的rviz',
        ),

        # 实车基础环境
        real_bringup,
        
        tf_to_odom_world,

        # 延迟启动 Ego 和轨迹跟踪器
        delayed_planner,
        delayed_tracker,

        # Ego RViz
        rviz,
    ])