import os
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.substitutions import LaunchConfiguration
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory

def generate_launch_description():
    # 1. 获取功能包路径
    # 导航配置包
    pkg_fishbot_nav = get_package_share_directory('fishbot_navigation2')
    # Nav2 官方包
    pkg_nav2_bringup = get_package_share_directory('nav2_bringup')
    # 机器人描述与仿真包 (用于启动 Gazebo)
    pkg_fishbot_desc = get_package_share_directory('fishbot_description')

    # 2. 配置文件路径定义
    # 默认地图路径 (优先使用 fishbot_navigation2 中的地图)
    default_map_path = os.path.join(pkg_fishbot_nav, 'maps', 'map.yaml')
    # 默认参数文件路径
    default_params_path = os.path.join(pkg_fishbot_nav, 'config', 'nav2_params.yaml')
    # RViz 配置文件 (使用 Nav2 默认配置)
    rviz_config_dir = os.path.join(pkg_nav2_bringup, 'rviz', 'nav2_default_view.rviz')

    # 3. 声明 Launch 参数 (允许用户在命令行覆盖)
    use_sim_time_arg = DeclareLaunchArgument(
        'use_sim_time',
        default_value='true',
        description='是否使用仿真时间 (Gazebo)'
    )
    
    map_arg = DeclareLaunchArgument(
        'map',
        default_value=default_map_path,
        description='SLAM 地图文件完整路径'
    )
    
    params_arg = DeclareLaunchArgument(
        'params_file',
        default_value=default_params_path,
        description='Nav2 参数文件完整路径'
    )

    # 获取配置好的参数对象
    use_sim_time = LaunchConfiguration('use_sim_time')
    map_path = LaunchConfiguration('map')
    params_file = LaunchConfiguration('params_file')

    # 4. 定义各个启动节点/动作

    # [动作 A] 启动 Gazebo 仿真环境 (包含小车模型)
    # 注意：确保 fishbot_description 包下有 launch/gazebo_sim.launch.py
    gazebo_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_fishbot_desc, 'launch', 'gazebo_sim.launch.py')
        ),
        launch_arguments={
            'use_sim_time': use_sim_time
        }.items()
    )

    # [动作 B] 启动 Nav2 导航栈
    nav2_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_nav2_bringup, 'launch', 'bringup_launch.py')
        ),
        launch_arguments={
            'map': map_path,
            'params_file': params_file,
            'use_sim_time': use_sim_time,
            'autostart': 'true'
        }.items()
    )

    # [节点 C] 启动红色物体跟随逻辑 (自定义业务逻辑)
    follow_node = Node(
        package='fishbot_navigation2',
        executable='red_object_follower.py',
        name='red_object_follower',
        output='screen',
        parameters=[{'use_sim_time': use_sim_time}]
    )

    # [节点 D] 启动红色物体键盘控制 (自定义业务逻辑)
    teleop_node = Node(
        package='fishbot_navigation2',
        executable='red_target_teleop.py',
        name='red_target_teleop',
        output='screen',
        parameters=[{'use_sim_time': use_sim_time}]
    )

    # [节点 E] 启动 RViz2 可视化
    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        arguments=[
            '-d', rviz_config_dir,
            '-f', 'map'  # 强制固定坐标系为 map，这对导航可视化很重要
        ],
        parameters=[{'use_sim_time': use_sim_time}],
        output='screen'
    )

    # 5. 组装并返回 LaunchDescription
    return LaunchDescription([
        # 参数声明
        use_sim_time_arg,
        map_arg,
        params_arg,
        # 启动动作
        gazebo_launch,      # 先启动仿真
        nav2_launch,        # 再启动导航
        # 功能节点
        follow_node,
        teleop_node,
        rviz_node           # 最后打开可视化
    ])
