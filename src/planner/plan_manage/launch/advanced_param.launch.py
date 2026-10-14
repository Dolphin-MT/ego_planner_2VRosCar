import os
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

def generate_launch_description():
    # LaunchConfigurations
    map_size_x = LaunchConfiguration('map_size_x_', default=42.0)
    map_size_y = LaunchConfiguration('map_size_y_', default=30.0)
    map_size_z = LaunchConfiguration('map_size_z_', default=5.0)
    
    odometry_topic = LaunchConfiguration('odometry_topic', default='odom')
    camera_pose_topic = LaunchConfiguration('camera_pose_topic', default='camera_pose')
    depth_topic = LaunchConfiguration('depth_topic', default='depth_image')
    cloud_topic = LaunchConfiguration('cloud_topic', default='cloud')
    
    cx = LaunchConfiguration('cx', default=321.04638671875)
    cy = LaunchConfiguration('cy', default=243.44969177246094)
    fx = LaunchConfiguration('fx', default=387.229248046875)
    fy = LaunchConfiguration('fy', default=387.229248046875)
    
    max_vel = LaunchConfiguration('max_vel', default=2.0)
    max_acc = LaunchConfiguration('max_acc', default=3.0)
    planning_horizon = LaunchConfiguration('planning_horizon', default=7.5)
    
    point_num = LaunchConfiguration('point_num', default=1)
    point0_x = LaunchConfiguration('point0_x', default=0.0)
    point0_y = LaunchConfiguration('point0_y', default=0.0)
    point0_z = LaunchConfiguration('point0_z', default=0.0)
    point1_x = LaunchConfiguration('point1_x', default=10.0)
    point1_y = LaunchConfiguration('point1_y', default=10.0)
    point1_z = LaunchConfiguration('point1_z', default=0.0)
    point2_x = LaunchConfiguration('point2_x', default=20.0)
    point2_y = LaunchConfiguration('point2_y', default=20.0)
    point2_z = LaunchConfiguration('point2_z', default=1.0)
    point3_x = LaunchConfiguration('point3_x', default=-10.0)
    point3_y = LaunchConfiguration('point3_y', default=-10.0)
    point3_z = LaunchConfiguration('point3_z', default=1.0)
    point4_x = LaunchConfiguration('point4_x', default=30.0)
    point4_y = LaunchConfiguration('point4_y', default=30.0)
    point4_z = LaunchConfiguration('point4_z', default=1.0)

    flight_type = LaunchConfiguration('flight_type', default=2)
    use_distinctive_trajs = LaunchConfiguration('use_distinctive_trajs', default=True)
    
    obj_num_set = LaunchConfiguration('obj_num_set', default=10)
    
    drone_id = LaunchConfiguration('drone_id', default=0)

    # DeclareLaunchArguments
    map_size_x_arg = DeclareLaunchArgument('map_size_x_', default_value=map_size_x, description='Map size along X')
    map_size_y_arg = DeclareLaunchArgument('map_size_y_', default_value=map_size_y, description='Map size along Y')
    map_size_z_arg = DeclareLaunchArgument('map_size_z_', default_value=map_size_z, description='Map size along Z')
    odometry_topic_arg = DeclareLaunchArgument('odometry_topic', default_value=odometry_topic, description='Odometry topic')
    camera_pose_topic_arg = DeclareLaunchArgument('camera_pose_topic', default_value=camera_pose_topic, description='Camera pose topic')
    depth_topic_arg = DeclareLaunchArgument('depth_topic', default_value=depth_topic, description='Depth topic')
    cloud_topic_arg = DeclareLaunchArgument('cloud_topic', default_value=cloud_topic, description='Point cloud topic')
    cx_arg = DeclareLaunchArgument('cx', default_value=cx, description='Camera intrinsic cx')
    cy_arg = DeclareLaunchArgument('cy', default_value=cy, description='Camera intrinsic cy')
    fx_arg = DeclareLaunchArgument('fx', default_value=fx, description='Camera intrinsic fx')
    fy_arg = DeclareLaunchArgument('fy', default_value=fy, description='Camera intrinsic fy')
    max_vel_arg = DeclareLaunchArgument('max_vel', default_value=max_vel, description='Maximum velocity')
    max_acc_arg = DeclareLaunchArgument('max_acc', default_value=max_acc, description='Maximum acceleration')
    planning_horizon_arg = DeclareLaunchArgument('planning_horizon', default_value=planning_horizon, description='Planning horizon')
    
    point_num_arg = DeclareLaunchArgument('point_num', default_value=point_num, description='Number of waypoints')
    point0_x_arg = DeclareLaunchArgument('point0_x', default_value=point0_x, description='Waypoint 0 X coordinate')#coordinate 坐标
    point0_y_arg = DeclareLaunchArgument('point0_y', default_value=point0_y, description='Waypoint 0 Y coordinate')
    point0_z_arg = DeclareLaunchArgument('point0_z', default_value=point0_z, description='Waypoint 0 Z coordinate')
    point1_x_arg = DeclareLaunchArgument('point1_x', default_value=point1_x, description='Waypoint 1 X coordinate')
    point1_y_arg = DeclareLaunchArgument('point1_y', default_value=point1_y, description='Waypoint 1 Y coordinate')
    point1_z_arg = DeclareLaunchArgument('point1_z', default_value=point1_z, description='Waypoint 1 Z coordinate')
    point2_x_arg = DeclareLaunchArgument('point2_x', default_value=point2_x, description='Waypoint 2 X coordinate')
    point2_y_arg = DeclareLaunchArgument('point2_y', default_value=point2_y, description='Waypoint 2 Y coordinate')
    point2_z_arg = DeclareLaunchArgument('point2_z', default_value=point2_z, description='Waypoint 2 Z coordinate')
    point3_x_arg = DeclareLaunchArgument('point3_x', default_value=point3_x, description='Waypoint 3 X coordinate')
    point3_y_arg = DeclareLaunchArgument('point3_y', default_value=point3_y, description='Waypoint 3 Y coordinate')
    point3_z_arg = DeclareLaunchArgument('point3_z', default_value=point3_z, description='Waypoint 3 Z coordinate')
    point4_x_arg = DeclareLaunchArgument('point4_x', default_value=point4_x, description='Waypoint 4 X coordinate')
    point4_y_arg = DeclareLaunchArgument('point4_y', default_value=point4_y, description='Waypoint 4 Y coordinate')
    point4_z_arg = DeclareLaunchArgument('point4_z', default_value=point4_z, description='Waypoint 4 Z coordinate')
    
    flight_type_arg = DeclareLaunchArgument('flight_type', default_value=flight_type, description='flight_type')
    #use_distinctive_trajs的作用：开了 True，无人机会变得“狡猾”且“圆滑”；关了 False，无人机像个“愣头青”，只认一条道走到黑。在你的集群任务里，保持 True 是解锁高密度编队的钥匙。
    use_distinctive_trajs_arg = DeclareLaunchArgument('use_distinctive_trajs', default_value=use_distinctive_trajs, description='Use distinctive trajectories')
    obj_num_set_arg = DeclareLaunchArgument('obj_num_set', default_value=obj_num_set, description='Number of objects')
    drone_id_arg = DeclareLaunchArgument('drone_id', default_value=drone_id, description='Drone ID')

    # Ego Planner Node 整个系统大脑，重点研读 作为启动器+状态机
    ego_planner_node = Node(
        package='ego_planner',
        executable='ego_planner_node',#最后把规划逻辑交给 EGOReplanFSM 去驱动
        name=['drone_', drone_id, '_ego_planner_node'],
        output='screen',
        remappings=[                #订阅的消息
            ('odom_world', ['drone_', drone_id, '_', odometry_topic]),#里程计，用来获取当前位置、速度、姿态
            ('planning/bspline', ['drone_', drone_id, '_planning/bspline']),#B-spline轨迹输出
            ('planning/data_display', ['drone_', drone_id, '_planning/data_display']),#调试数据可视化
            ('planning/broadcast_bspline_from_planner', '/broadcast_bspline'),#集群轨迹广播发送
            ('planning/broadcast_bspline_to_planner', '/broadcast_bspline'),#集群轨迹广播接收
            #Rviz可视化数据
            ('goal_point', ['drone_', drone_id, '_plan_vis/goal_point']),
            ('global_list', ['drone_', drone_id, '_plan_vis/global_list']),
            ('init_list', ['drone_', drone_id, '_plan_vis/init_list']),
            ('optimal_list', ['drone_', drone_id, '_plan_vis/optimal_list']),
            ('a_star_list', ['drone_', drone_id, '_plan_vis/a_star_list']),
            #珊格地图输入：里程计 点云 相机位姿 深度图
            ('grid_map/odom', ['drone_', drone_id, '_', odometry_topic]),
            ('grid_map/cloud', ['drone_', drone_id, '_', cloud_topic]),
            ('grid_map/pose', ['drone_', drone_id, '_', camera_pose_topic]),
            ('grid_map/depth', ['drone_', drone_id, '_', depth_topic]),
            ('grid_map/occupancy_inflate', ['drone_', drone_id, '_grid/grid_map/occupancy_inflate']) #occupancy_inflate 膨胀后的占据地图输出
        ],
        # fsm是是有限状态机的意思
        parameters=[#需要着重理解这里的代码，知道变量的意思
            {'fsm/flight_type': flight_type},#飞行类型 1=手动目标点(RViz)，2=预设航点，3=参考路径
            {'fsm/thresh_replan_time': 1.0},#重规划的超时时间 
            {'fsm/thresh_no_replan_meter': 1.0},#不需要规划的阈值
            {'fsm/planning_horizon': planning_horizon},#规划的范围
            {'fsm/planning_horizen_time': 3.0},        #轨迹最长持续时间
            {'fsm/emergency_time': 1.0},               #碰撞时间<1s，触发悬停功能
            {'fsm/realworld_experiment': False},        #真机实验标志
            {'fsm/fail_safe': True},                    #故障保护
            
            {'fsm/waypoint_num': point_num},    #航点数量
            {'fsm/waypoint0_x': point0_x},
            {'fsm/waypoint0_y': point0_y},
            {'fsm/waypoint0_z': point0_z},
            {'fsm/waypoint1_x': point1_x},
            {'fsm/waypoint1_y': point1_y},
            {'fsm/waypoint1_z': point1_z},
            {'fsm/waypoint2_x': point2_x},
            {'fsm/waypoint2_y': point2_y},
            {'fsm/waypoint2_z': point2_z},
            {'fsm/waypoint3_x': point3_x},
            {'fsm/waypoint3_y': point3_y},
            {'fsm/waypoint3_z': point3_z},
            {'fsm/waypoint4_x': point4_x},
            {'fsm/waypoint4_y': point4_y},
            {'fsm/waypoint4_z': point4_z},

            #基础地图参数
            {'grid_map/resolution': 0.1},   #分辨率
            {'grid_map/map_size_x': map_size_x},
            {'grid_map/map_size_y': map_size_y},
            {'grid_map/map_size_z': map_size_z},
            {'grid_map/local_update_range_x': 5.5},#局部更新范围
            {'grid_map/local_update_range_y': 5.5},
            {'grid_map/local_update_range_z': 4.5},
            {'grid_map/obstacles_inflation': 0.099}, #障碍物膨胀半径 0.099
            {'grid_map/local_map_margin': 10},#地图边界余量
            {'grid_map/ground_height': -0.01},#地面高度
            # camera parameter
            {'grid_map/cx': cx},
            {'grid_map/cy': cy},
            {'grid_map/fx': fx},
            {'grid_map/fy': fy},
            # depth filter 深度滤波参数
            {'grid_map/use_depth_filter': True},#是否启用
            {'grid_map/depth_filter_tolerance': 0.15},#相机深度容差
            {'grid_map/depth_filter_maxdist': 5.0},
            {'grid_map/depth_filter_mindist': 0.2},
            {'grid_map/depth_filter_margin': 2}, #图像边缘忽略
            {'grid_map/k_depth_scaling_factor': 1000.0},#深度缩放因子 深度图单位转换
            {'grid_map/skip_pixel': 2},#像素降采样
            # local fusion 局部数据融合
            {'grid_map/p_hit': 0.65},#命中概率
            {'grid_map/p_miss': 0.35},#未命中
            {'grid_map/p_min': 0.12},#最小概率
            {'grid_map/p_max': 0.90},
            {'grid_map/p_occ': 0.80},#占据阈值 检测某个像素点是否是障碍
            {'grid_map/min_ray_length': 0.1},#最小射线长度 用于删除一些无效射线
            {'grid_map/max_ray_length': 4.5},

            #其他地图参数
            {'grid_map/virtual_ceil_height': 2.9},#虚拟天花板高度
            {'grid_map/visualization_truncate_height': 1.8},#可视化截断高度 RViz 显示时只显示低于此高度的障碍物
            {'grid_map/show_occ_time': False},#显示占用时间
            {'grid_map/pose_type': 1},#位姿类型 1=Odom，2=Pose
            {'grid_map/frame_id': "world"},#坐标系
            # planner manager
            {'manager/max_vel': max_vel},
            {'manager/max_acc': max_acc},
            {'manager/max_jerk': 4.0},
            {'manager/control_points_distance': 0.4},#控制点间距 相邻 B-spline 控制点距离 0.4m
            {'manager/feasibility_tolerance': 0.05},#可行性容差
            {'manager/planning_horizon': planning_horizon},
            {'manager/use_distinctive_trajs': use_distinctive_trajs},   #拓扑多样化轨迹
            {'manager/drone_id': drone_id},
            # Trajectory optimization parameters 轨迹优化参数
            # 总代价函数：J = λ_smooth·J_smooth + λ_collision·J_collision + 
            # λ_feasibility·J_feasibility + λ_fitness·J_fitness
            {'optimization/lambda_smooth': 1.0}, #lambda:平滑系数 再看一下课程里面的轨迹优化内容
            {'optimization/lambda_collision': 0.5},#碰撞代价权重
            {'optimization/lambda_feasibility': 0.1},#动力学可行性权重
            {'optimization/lambda_fitness': 1.0}, #拟合权重
            {'optimization/dist0': 0.5},        #安全距离 距离障碍物 < 0.5m 时开始产生碰撞代价
            {'optimization/swarm_clearance': 0.5},#集群安全距离 与其他无人机的最小距离
            {'optimization/max_vel': max_vel},
            {'optimization/max_acc': max_acc},

            # B-Spline parameters
            {'bspline/limit_vel': max_vel},
            {'bspline/limit_acc': max_acc},
            {'bspline/limit_ratio': 1.1}, #限制比率

            # Object prediction parameters 仅在 use_dynamic:=True 时启用。
            {'prediction/obj_num': obj_num_set},
            {'prediction/lambda': 1.0}, #预测衰减系数
            {'prediction/predict_rate': 1.0}#预测频率
        ]
    )

    # Create LaunchDescription
    ld = LaunchDescription()

    # Add LaunchArguments
    ld.add_action(map_size_x_arg)
    ld.add_action(map_size_y_arg)
    ld.add_action(map_size_z_arg)
    ld.add_action(odometry_topic_arg)
    ld.add_action(camera_pose_topic_arg)
    ld.add_action(depth_topic_arg)
    ld.add_action(cloud_topic_arg)
    ld.add_action(cx_arg)
    ld.add_action(cy_arg)
    ld.add_action(fx_arg)
    ld.add_action(fy_arg)
    ld.add_action(max_vel_arg)
    ld.add_action(max_acc_arg)
    ld.add_action(planning_horizon_arg)
    
    ld.add_action(point_num_arg)
    ld.add_action(point0_x_arg)
    ld.add_action(point0_y_arg)
    ld.add_action(point0_z_arg)
    ld.add_action(point1_x_arg)
    ld.add_action(point1_y_arg)
    ld.add_action(point1_z_arg)
    ld.add_action(point2_x_arg)
    ld.add_action(point2_y_arg)
    ld.add_action(point2_z_arg)
    ld.add_action(point3_x_arg)
    ld.add_action(point3_y_arg)
    ld.add_action(point3_z_arg)
    ld.add_action(point4_x_arg)
    ld.add_action(point4_y_arg)
    ld.add_action(point4_z_arg)
    
    ld.add_action(flight_type_arg)
    ld.add_action(use_distinctive_trajs_arg)
    ld.add_action(obj_num_set_arg)
    ld.add_action(drone_id_arg)


    # Add Node
    ld.add_action(ego_planner_node)

    return ld
