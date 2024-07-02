# Quaternion-based 9-DOF IMU Kalman Filter in ROS2 Foxy

This work presents an orientation estimation using a quaternion-based Kalman filter with a 9-DOF IMU in ROS2 foxy.

<p align = "left">
 <img src = "https://github.com/SeonilChoi/Quaternion-based-9-DOF-IMU-Kalman-Filter-in-ROS2-Foxy/assets/172185042/616a6027-c3df-443f-a143-0f3f420fabad" />
</p>

 :warning: The yaw is only valid when the IMU sensor is horizontal or near horizontal, because the magnetometer is affected by the surrounding magnetic field. :warning:

> [!NOTE]
> This work does not include Arduino code that sends raw IMU data to a PC using Serial communication. The Arduino code is [here](https://github.com/SeonilChoi/MPU-9250-Serial-Communication.git).

# Description

The data from the IMU, which consists of accelerations, angular velocities, and magnetic fields, is published by the node named **IMUPublisher**.

This package reads raw data from the IMU using a library named **PortHandler**.

The node named **IMUKalmanFilterPublisher** subscribes to the topic where IMU data is published and estimates the orientation using a Kalman filter.

The library named **KalmanFilter** calculate the Kalman gain and update variables.

# Usage

## Build the package

```
colcon build --packages-select imu_kalman_filter
```

## Run

```
ros2 launch imu_kalman_filter imu_kalman_filter.launch.py
```
