import os
import datetime

from ament_index_python import get_package_share_directory

from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, ExecuteProcess
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node

#  + datetime.date.today().strftime("%d_%B_%Y_%I_%M%p")
bag_file_path = os.path.expanduser('~/Documents/emlid_lidar_test')

livox_ros2_driver_path = get_package_share_directory('livox_ros2_driver')
nmea_navsat_driver_path = get_package_share_directory('nmea_navsat_driver')

livox_lidar_rviz_launch = IncludeLaunchDescription(
  PythonLaunchDescriptionSource(
    livox_ros2_driver_path + '/launch/livox_lidar_rviz_launch.py'
  )
)

nmea_serial_driver_launch = IncludeLaunchDescription(
  PythonLaunchDescriptionSource(
    nmea_navsat_driver_path + '/launch/nmea_serial_driver.launch.py'
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
    nmea_serial_driver_launch,
    # rosbag_record

    ExecuteProcess(
      cmd=['ros2', 'bag', 'record', '-a', '-o', bag_file_path],
      output='screen'
    ),
  ])