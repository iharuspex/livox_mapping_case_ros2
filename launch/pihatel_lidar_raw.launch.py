import os
import datetime

from ament_index_python import get_package_share_directory

from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, ExecuteProcess
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node


bag_file_path = os.path.expanduser('~/Documents/pihatel_lidar_test')
bag_file_path += datetime.datetime.now().strftime("%d-%M-%y_%H-%M-%S")

livox_ros2_driver_path = get_package_share_directory('livox_ros2_driver')
gnss_driver_path = get_package_share_directory('pihatel_ros2_driver')

livox_lidar_rviz_launch = IncludeLaunchDescription(
  PythonLaunchDescriptionSource(
    livox_ros2_driver_path + '/launch/livox_lidar_rviz_launch.py'
  )
)

gnss_driver_launch = IncludeLaunchDescription(
  PythonLaunchDescriptionSource(
    gnss_driver_path + '/launch/pihatel_driver.launch.py'
  )
)


def generate_launch_description():
  rosbag_record = Node(
      package='rosbag2_transport',
      executable='recorder',
      name='rosbag_record',
      output='screen',
      arguments=['-a', '-o', bag_file_path]
  )

  return LaunchDescription([
    livox_lidar_rviz_launch,
    gnss_driver_launch,

    ExecuteProcess(
      cmd=['ros2', 'bag', 'record', '-a', '-o', bag_file_path],
      output='screen'
    ),
  ])