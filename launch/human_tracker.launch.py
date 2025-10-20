from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from ament_index_python.packages import get_package_share_directory
import os

def generate_launch_description():
    pkg_dir = get_package_share_directory('human_tracker_ros2')
    
    detector_config = os.path.join(pkg_dir, 'config', 'detector_params.yaml')
    tracker_config = os.path.join(pkg_dir, 'config', 'tracker_params.yaml')
    
    # Launch arguments
    use_sim_time = LaunchConfiguration('use_sim_time', default='false')
    
    return LaunchDescription([
        DeclareLaunchArgument(
            'use_sim_time',
            default_value='false',
            description='Use simulation time if true'
        ),
        
        # Human detector node
        Node(
            package='human_tracker_ros2',
            executable='detector_node',
            name='human_detector',
            output='screen',
            parameters=[
                detector_config,
                {'use_sim_time': use_sim_time}
            ],
            remappings=[
                ('input/pointcloud', '/livox/lidar')
            ]
        ),
        
        # Human tracker node
        Node(
            package='human_tracker_ros2',
            executable='tracker_node',
            name='human_tracker',
            output='screen',
            parameters=[
                tracker_config,
                {'use_sim_time': use_sim_time}
            ]
        )
    ])