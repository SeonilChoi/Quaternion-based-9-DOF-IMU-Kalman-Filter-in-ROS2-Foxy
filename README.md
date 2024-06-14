# Quaternion-based-9-DOF-IMU-Kalman-Filter-in-ROS2-foxy

This work presents an orientation estimation using a quaternion-based Kalman filter with a 9-DOF IMU in ROS2 foxy.

# Description

The data from the IMU, which consists of accelerations, angular velocities, and magnetic fields, is published by the package named **imu_publisher**.

This package reads raw data from the IMU using source codes named **imu_packet.hpp** and **imu_packet.cpp**.

The package named **imu_kalman** subscribes to the topic where IMU data is published and estimates the orientation using a Kalman filter.

:warning: The source codes named **kalman_filter.hpp** and **kalman_filter.cpp** calculate the Kalman gain and update variables.

# Usage

## Build the package

```
cd ~/colcon_ws
colcon build
```

## Run

```
ros2 run imu_kalman_filter imu_publisher
ros2 run imu_kalman_filter imu_kalman
```
