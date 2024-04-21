import os
import datetime
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import ExecuteProcess

def generate_launch_description():

    config = os.path.join(
        get_package_share_directory('pure_pursuit'),
                                    'config',
                                    'params.yaml')

    param_node = Node(
        package='pure_pursuit',
        executable='pursuit_node',
        output='screen',
        parameters=[config]
    )

    bag_output_dir = os.path.expanduser('~/rosbag_data/lab7/')
    timestamp = datetime.datetime.now().strftime("%Y-%m-%d_%H-%M-%S")
    bag_filename = f'{bag_output_dir}/rosbag_{timestamp}'
    
    ros_bag_node = ExecuteProcess(
        cmd=['gnome-terminal', '--', 'bash', '-c', f'ros2 bag record -o {bag_filename} /positionX /positionY'],
        output='screen',
    )

    l_d = LaunchDescription([param_node, ros_bag_node])
    return l_d