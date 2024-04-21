import os
import datetime
from launch_ros.actions import Node
from launch import LaunchDescription
from launch.actions import ExecuteProcess

def generate_launch_description():
    micro_ros_node = ExecuteProcess(
        cmd=['gnome-terminal', '--', 'bash', '-c', 'ssh puzzlebot@10.42.0.1'],
        output='screen'
    )

    odometry_node = Node(
        package='odometria_package',
        executable='odometria',
        output='screen'
    )

    teleop_node = ExecuteProcess(
        cmd=['gnome-terminal', '--', 'bash', '-c', 'ros2 run teleop_twist_keyboard teleop_twist_keyboard'],
        output='screen'
    )

    bag_output_dir = os.path.expanduser('~/rosbag_data/lab6/')
    timestamp = datetime.datetime.now().strftime("%Y-%m-%d_%H-%M-%S")
    bag_filename = f'{bag_output_dir}/rosbag_{timestamp}'
    
    ros_bag_node = ExecuteProcess(
        cmd=['gnome-terminal', '--', 'bash', '-c', f'ros2 bag record -o {bag_filename} /positionX /positionY'],
        output='screen',
    )

    ld = LaunchDescription([micro_ros_node, odometry_node, teleop_node, ros_bag_node])
    return ld
