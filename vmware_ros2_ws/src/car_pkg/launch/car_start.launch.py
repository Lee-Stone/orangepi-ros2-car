import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, TimerAction
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PythonExpression
from launch_ros.actions import Node


def generate_launch_description():
    package_dir = os.path.dirname(os.path.dirname(os.path.realpath(__file__)))
    urdf_path = os.path.join(package_dir, 'urdf', 'car_model.urdf')
    with open(urdf_path, 'r') as f:
        robot_description = f.read()

    config_dir = os.path.join(package_dir, 'config')
    nav2_params = os.path.join(config_dir, 'nav2_params.yaml')
    map_path = os.path.join(package_dir, 'maps', 'map1.yaml')
    slam_rviz = os.path.join(package_dir, 'rviz', 'slam.rviz')
    nav2_rviz = os.path.join(package_dir, 'rviz', 'nav2.rviz')

    # 可选择建图或导航，默认保持原来的建图模式
    mode = LaunchConfiguration('mode')
    mode_arg = DeclareLaunchArgument(
        'mode',
        default_value='slam',
        description='slam: 启动建图; nav: 启动Nav2自主导航',
    )
    slam_condition = IfCondition(
        PythonExpression(["'", mode, "' == 'slam'"])
    )
    nav_condition = IfCondition(
        PythonExpression(["'", mode, "' == 'nav'"])
    )

    # 发布小车模型，提供base_link到laser_link的固定TF
    model = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        parameters=[{'robot_description': robot_description, 'publish_frequency': 50.0}],
        output='screen',
    )

    # 建图模式下的遥控和Cartographer节点
    key_node = Node(
        package='car_pkg',
        executable='key_node',
        name='key_node',
        prefix='gnome-terminal --wait --title=key_node --',
        output='screen',
        condition=slam_condition,
    )
    cartographer = Node(
        package='cartographer_ros',
        executable='cartographer_node',
        name='cartographer_node',
        output='screen',
        arguments=[
            '-configuration_directory', config_dir,
            '-configuration_basename', 'car_2d.lua',
        ],
        condition=slam_condition,
    )
    grid = Node(
        package='cartographer_ros',
        executable='cartographer_occupancy_grid_node',
        name='cartographer_occupancy_grid_node',
        output='screen',
        parameters=[{'resolution': 0.05}],
        condition=slam_condition,
    )

    # 导航模式加载保存的地图、AMCL定位和Nav2规划控制节点
    nav2 = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(get_package_share_directory('nav2_bringup'), 'launch', 'bringup_launch.py')
        ),
        launch_arguments={
            'map': map_path,
            'use_sim_time': 'false',
            'params_file': nav2_params,
            'autostart': 'True',
            'use_composition': 'False',
            'use_localization': 'True',
        }.items(),
        condition=nav_condition,
    )

    # 两种模式使用不同的RViz配置
    rviz_slam = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        arguments=['-d', slam_rviz],
        output='screen',
        condition=slam_condition,
    )
    rviz_nav = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        arguments=['-d', nav2_rviz],
        output='screen',
        condition=nav_condition,
    )

    return LaunchDescription([
        mode_arg,
        model,
        key_node,
        TimerAction(period=3.0, actions=[cartographer, grid, rviz_slam]),
        TimerAction(period=2.0, actions=[nav2]),
        TimerAction(period=3.0, actions=[rviz_nav]),
    ])
