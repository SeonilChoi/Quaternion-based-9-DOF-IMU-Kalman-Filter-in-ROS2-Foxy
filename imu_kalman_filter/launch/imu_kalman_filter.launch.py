import os

from launch import LaunchDescription
from launch_ros.actions import Node

from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource

from ament_index_python.packages import get_package_share_directory

def generate_launch_description():
    imu_serial_communication_pkg_path = os.path.join(
        get_package_share_directory('imu_serial_communication')
    )
    
    imu_serial_communication_launcher = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(
                imu_serial_communication_pkg_path, 'launch', 'imu_serial_communication.launch.py'
            )
        )
    )

    imu_kalman_filter_publisher = Node(
        package = "imu_kalman_filter",
        executable = "imu_kalman_filter_publisher",
        name = "imu_kalman_filter_publisher",
        parameters = [{
            "rolling_mean":True,
            "rolling_window_size":5
        }]
    )
    
    return LaunchDescription([
        imu_serial_communication_launcher,
        imu_kalman_filter_publisher
    ])
    
