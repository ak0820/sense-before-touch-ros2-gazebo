from pathlib import Path
from launch import LaunchDescription
from launch.actions import ExecuteProcess, TimerAction
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory


def generate_launch_description():
    world = str(Path(get_package_share_directory('sense_before_touch_ros2')) / 'worlds' / 'rescue_warehouse.sdf')
    return LaunchDescription([
        ExecuteProcess(cmd=['gz', 'sim', '-s', '-r', '-v', '3', world], output='screen'),
        Node(
            package='ros_gz_bridge',
            executable='parameter_bridge',
            name='warehouse_bridge',
            output='screen',
            arguments=[
                '/model/rescue_rover/cmd_vel@geometry_msgs/msg/Twist]gz.msgs.Twist',
                '/model/rescue_rover/ground_truth@nav_msgs/msg/Odometry[gz.msgs.Odometry',
            ],
        ),
        TimerAction(period=8.0, actions=[
            Node(package='sense_before_touch_ros2', executable='controller',
                 name='rescue_ai', output='screen')
        ]),
    ])
