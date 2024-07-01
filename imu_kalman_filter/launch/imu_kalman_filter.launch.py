from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    imu_publisher = Node(
        package = "imu_kalman_filter",
        executable = "imu_publisher",
        name = "imu_publisher"
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
        imu_publisher,
        imu_kalman_filter_publisher
    ])
    
