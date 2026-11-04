# car_start.launch.py：一次启动IMU、轮式里程计、EKF和PID控制节点
# key_node运行在虚拟机，不放进香橙派launch
import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, TimerAction
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node


def generate_launch_description():
    launch_dir = os.path.dirname(os.path.realpath(__file__))
    # ekf.yaml放在src/yaml，和launch文件不在同一目录，需要先回到上一级再进入yaml
    config = os.path.join(launch_dir, '..', 'yaml', 'ekf.yaml')

    # 激光雷达先启动，建图节点启动时才有/scan可以订阅
    lidar = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(get_package_share_directory('cspc_lidar'), 'launch', 'lidar_launch.py')
        )
    )

    # IMU在激光雷达之后启动
    imu = Node(
        package='car_pkg',
        executable='imu_node',
        name='imu_node',
        output='screen',
    )
    # 编码器要打开/dev/gpiochip0并启动读取线程，延后1秒再启动
    odom = Node(
        package='car_pkg',
        executable='odom_node',
        name='odom_node',
        output='screen',
    )
    # EKF在IMU和轮式odom之后启动，此时两个输入话题都已发布
    ekf = Node(
        package='robot_localization',
        executable='ekf_node',
        name='ekf_filter_node',
        output='screen',
        parameters=[config],
        remappings=[('odometry/filtered', '/odom')],
    )
    # PID控制节点要等/odom稳定后再启动
    vel = Node(
        package='car_pkg',
        executable='vel_node',
        name='vel_node',
        output='screen',
    )

    return LaunchDescription([
        lidar,
        imu,
        TimerAction(period=1.0, actions=[odom]),
        TimerAction(period=3.0, actions=[ekf]),
        TimerAction(period=4.0, actions=[vel]),
    ])
