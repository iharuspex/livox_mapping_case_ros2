import os


from ament_index_python import get_package_share_directory

from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, ExecuteProcess
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node

rviz_config_path = os.path.join(
  get_package_share_directory('livox_mapping'), 'livox_mapping.rviz'
)

def generate_launch_description():
  livox_mapping_case = Node(
    package='livox_mapping',
    executable='livox_mapping_case',
    output='screen',
    # prefix='gdbserver localhost:3000',
    name='livox_mapping_case',
    parameters=[{
      'map_file_path': '/home/haruspex/Documents/'
    }]
  )

  livox_rviz = Node(
      package='rviz2',
      executable='rviz2',
      name='rviz',
      output='screen',
      arguments=['--display-config', rviz_config_path]
  )

  return LaunchDescription([
    livox_mapping_case,
    livox_rviz
  ])