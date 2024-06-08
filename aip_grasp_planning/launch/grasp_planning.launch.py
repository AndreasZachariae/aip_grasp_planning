from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import ThisLaunchFileDir
import os
from ament_index_python.packages import get_package_share_directory

def generate_launch_description():
    workspace_dir = get_package_share_directory('point_transformation')
    point_transformation_launch_file = os.path.join(workspace_dir, 'launch', 'point_transformation.launch.py')

    return LaunchDescription([
        Node(
            package='aip_grasp_planning',
            executable='point_cloud_processing_node',
            name='point_cloud_processing_node'
        ),
        Node(
            package='aip_grasp_planning',
            executable='grasp_planning_node.py',
            name='grasp_planning_node'
        ),
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(point_transformation_launch_file)
        )
    ])
