import os
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from ament_index_python.packages import get_package_share_directory
from launch.substitutions import PythonExpression
from launch.conditions import IfCondition, UnlessCondition

def generate_launch_description():
    # 关于 LaunchConfiguration与 DeclareLaunchArgument 的介绍，以odom_topic为例子
    # 1. 先声明（定义接口 DeclareLaunchArgument 形参定义）：告诉系统，这个 launch 有个参数叫 odom_topic，默认是 visual_slam/odom
    # 2. 后配置（获取实际值LaunchConfiguration 世纪传入的实参变量）：创建一个变量，用于在运行时获取这个参数的真实值
    # 这里顺序反了，不过也能用
    # 定义参数的 LaunchConfiguration
    obj_num = LaunchConfiguration('obj_num', default=10)#障碍物的数量 作为obj_num_set在advanced_param.launch.py中传入
    drone_id = LaunchConfiguration('drone_id', default=0)#无人机编号
    #定义地图尺寸 设置地图场景的大小
    map_size_x = LaunchConfiguration('map_size_x', default = 50.0)
    map_size_y = LaunchConfiguration('map_size_y', default = 25.0)
    map_size_z = LaunchConfiguration('map_size_z', default = 2.0)
    # 传给 simulator/其它节点作为里程计输入主题（在包含的 launch 中以 odometry_topic 传递）。
    odom_topic = LaunchConfiguration('odom_topic', default = 'visual_slam/odom')#里程记主题名
    
    # 声明全局参数
    obj_num_cmd = DeclareLaunchArgument('obj_num', default_value=obj_num, description='Number of objects')
    drone_id_cmd = DeclareLaunchArgument('drone_id', default_value=drone_id, description='Drone ID')
    
    map_size_x_cmd = DeclareLaunchArgument('map_size_x', default_value=map_size_x, description='Map size along x')
    map_size_y_cmd = DeclareLaunchArgument('map_size_y', default_value=map_size_y, description='Map size along y')
    map_size_z_cmd = DeclareLaunchArgument('map_size_z', default_value=map_size_z, description='Map size along z')
    odom_topic_cmd = DeclareLaunchArgument('odom_topic', default_value=odom_topic, description='Odometry topic')#odometry:里程计

    # 是一个布尔开关 决定使用哪种地图源：False 时启用 map_generator 节点，True 时启用 mockamap 节点 
    use_mockamap = LaunchConfiguration('use_mockamap', default=False) # map_generator or mockamap 
    
    use_mockamap_cmd = DeclareLaunchArgument('use_mockamap', default_value=use_mockamap, description='Choose map type, map_generator or mockamap')
    # 布尔开关 决定仿真器是否开启动力学仿真 是否要动态避障
    use_dynamic = LaunchConfiguration('use_dynamic', default=False)  
    use_dynamic_cmd = DeclareLaunchArgument('use_dynamic', default_value=use_dynamic, description='Use Drone Simulation Considering Dynamics or Not')
    
    # map 地图生成节点 生成 Random Forest（随机森林） 
    map_generator_node = Node(
        package='map_generator',
        executable='random_forest',
        name='random_forest',
        output='screen',
        parameters=[
            # 参数更改方法：例如map/resolution
            # ros2 launch <package> single_run_in_sim.launch.py map/resolution:=0.05
            # 地图大小
            {'map/x_size': 26.0},
            {'map/y_size': 20.0},
            {'map/z_size': 3.0},
            {'map/resolution': 0.1},#地图方格的分辨率0.1m/格
            #障碍物信息
            {'ObstacleShape/seed': 1.0},#随机种子1,用于可重复的随机障碍生成
            {'map/obs_num': 200},#250
            {'ObstacleShape/lower_rad': 0.5},#障碍物的半径下界0.5
            {'ObstacleShape/upper_rad': 0.8},#上界0.7
            {'ObstacleShape/lower_hei': 0.0},#高度下界0.0
            {'ObstacleShape/upper_hei': 3.0},#高度上界3.0
            {'map/circle_num': 250},#250
            {'ObstacleShape/radius_l': 0.8},#圆环半径上下界0.7
            {'ObstacleShape/radius_h': 1.0},#0.5
            {'ObstacleShape/z_l': 0.7},#垂直方向障碍物尺寸界限0.7
            {'ObstacleShape/z_h': 0.8},#0.8
            {'ObstacleShape/theta': 0.5},#0.5
            {'pub_rate': 1.0},#节点发布频率
            {'min_distance': 1.0}#障碍物之间的最段距离0.8
            #改变地图信息
        ],
        condition = UnlessCondition(use_mockamap)
    )
    # 另一个地图生成节点，当use_mockamap为true时启用 生成 复杂室内/迷宫环境
    mockamap_node = Node(
        package='mockamap',
        executable='mockamap_node',
        name='mockamap_node',
        output='screen',
        remappings=[
            ('/mock_map', '/map_generator/global_cloud')
        ],
        parameters=[
            {'seed': 127},
            {'update_freq': 0.5},
            {'resolution': 0.1},
            {'x_length': PythonExpression(['int(', map_size_x, ')'])},
            {'y_length': PythonExpression(['int(', map_size_y, ')'])},
            {'z_length': PythonExpression(['int(', map_size_z, ')'])},
            {'type': 1},
            {'complexity': 0.05},
            {'fill': 0.12},
            {'fractal': 1},
            {'attenuation': 0.1}
        ],
        condition = IfCondition(use_mockamap)
    )
    
    # Include advanced parameters 启动 launch 文件 advanced_param.launch.py 是整个系统的大脑
    advanced_param_include = IncludeLaunchDescription(
        # “去加载 ego_planner 包里的 advanced_param.launch.py”。
        PythonLaunchDescriptionSource(os.path.join(
            get_package_share_directory('ego_planner'), 'launch', 'advanced_param.launch.py')),
        launch_arguments={
            'drone_id': drone_id,
            'map_size_x_': map_size_x,
            'map_size_y_': map_size_y,
            'map_size_z_': map_size_z,
            'odometry_topic': odom_topic,#里程计话题名
            'obj_num_set': obj_num,
            # 相机位姿、深度图、点云话题。
            # 这些数据会被规划器和地图模块用于构建障碍物地图
            'camera_pose_topic': 'pcl_render_node/camera_pose',
            'depth_topic': 'pcl_render_node/depth',
            'cloud_topic': 'pcl_render_node/cloud',
            # 相机内部参数
            'cx': str(321.04638671875),
            'cy': str(243.44969177246094),
            'fx': str(387.229248046875),
            'fy': str(387.229248046875),
            'max_vel': str(2.0),
            'max_acc': str(6.0),
            'planning_horizon': str(7.5),#规划的视野范围
            'use_distinctive_trajs': 'True',#是否生成更“有区别”的多条轨迹。
            'flight_type': str(2),#飞行类型，决定规划器如何处理目标点和任务模式
            'point_num': str(4),#预设航点 即为依次在四个点之间飞行
            'point0_x': str(15.0),#15
            'point0_y': str(7.0),#0
            'point0_z': str(2.0),#1.0
            
            'point1_x': str(-15.0),#-15
            'point1_y': str(-7.0),
            'point1_z': str(1.5),#1.0
            
            'point2_x': str(15.0),#15
            'point2_y': str(-7.0),
            'point2_z': str(0.5),#1.0
            
            'point3_x': str(-15.0),
            'point3_y': str(0.0),
            'point3_z': str(1.0),
            
            'point4_x': str(15.0),
            'point4_y': str(0.0),
            'point4_z': str(1.0),
        }.items()
    )
# .items() 是把优雅的字典写法，强制转换成 Launch API 所需的标准元组列表格式的“桥梁”。
# 记忆口诀：只要看到 launch_arguments={ ... }，后面必须紧跟 .items()；如果不想写 .items()，就用 [ ( ... ) ] 的列表形式。

    # Trajectory server node 轨迹执行服务节点
    traj_server_node = Node(
        package='ego_planner',
        executable='traj_server',
        name=['drone_', drone_id, '_traj_server'],#节点起一个实例名称例如你可能启动多个无人机时，就能区分成：drone_0_traj_server
        output='screen',
        remappings=[#话题重映射 原本默认的接口名，改成你指定的新名字。
            # 例如把position_cmd重映射为drone_0_planning/pos_cmd
            ('position_cmd', ['drone_', drone_id, '_planning/pos_cmd']),
            ('planning/bspline', ['drone_', drone_id, '_planning/bspline'])
        ],
        parameters=[
            {'traj_server/time_forward': 1.0}#轨迹前向预测时间，提前发送控制指令
        ]
    )
    
    # Include simulator 仿真器节点 模拟无人机动力学 传感器 里程计的仿真
    simulator_include = IncludeLaunchDescription(PythonLaunchDescriptionSource(
        os.path.join(get_package_share_directory('ego_planner'), 'launch', 'simulator.launch.py')),
        launch_arguments={
            'use_dynamic': use_dynamic,
            'drone_id': drone_id,
            'map_size_x_': map_size_x,
            'map_size_y_': map_size_y,
            'map_size_z_': map_size_z,
            'init_x_': str(-15.0),
            'init_y_': str(0.0),
            'init_z_': str(0.1),
            'odometry_topic': odom_topic
        }.items()
    )

    ld = LaunchDescription()
    
    ld.add_action(map_size_x_cmd)
    ld.add_action(map_size_y_cmd)
    ld.add_action(map_size_z_cmd)
    ld.add_action(odom_topic_cmd)
    ld.add_action(obj_num_cmd)
    ld.add_action(drone_id_cmd)
    ld.add_action(use_dynamic_cmd)
    ld.add_action(use_mockamap_cmd)

    # 添加 Map Generator 节点
    ld.add_action(map_generator_node)
    ld.add_action(mockamap_node)
    ld.add_action(advanced_param_include)
    ld.add_action(traj_server_node)
    ld.add_action(simulator_include)

    return ld
