# Quaternion based 9 DOF IMU Kalman Filter in ROS2 Foxy

This work presents an orientation estimation using a quaternion-based Kalman filter with a 9-DOF IMU in ROS2 foxy.

> [!NOTE]
> This work does not include Arduino code that sends raw IMU data to a PC using Serial communication. The Arduino code is [here](https://github.com/SeonilChoi/MPU-9250-Serial-Communication.git).

# Description

The data from the IMU, which consists of accelerations, angular velocities, and magnetic fields, is published by the node named **IMUPublisher**.

This package reads raw data from the IMU using a library named **PortHandler**.

The node named **IMUKalman** subscribes to the topic where IMU data is published and estimates the orientation using a Kalman filter.

The library named **KalmanFilter** calculate the Kalman gain and update variables.

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
