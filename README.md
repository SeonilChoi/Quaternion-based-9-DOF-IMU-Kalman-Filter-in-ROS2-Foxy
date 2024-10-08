# Quaternion-based 9-DOF IMU Kalman Filter in ROS2 Foxy

This work presents an orientation estimation using a quaternion-based Kalman filter with a 9-DOF IMU in ROS2 foxy.

<p align = "left">
 <img src = "https://github.com/user-attachments/assets/8a27cac1-5f29-4b08-b172-7c73ea0b62c0" width=750/>
</p>

 :warning: The yaw(heading) is only valid when the IMU sensor is horizontal or near horizontal.

> [!NOTE]
> This work does not include sending raw IMU data to a PC or receiving it using serial communication. If you need this, you can check the following [link](https://github.com/SeonilChoi/MPU-9250-Serial-Communication-in-ROS2-Foxy.git).

# Description

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

or

```
ros2 launch imu_kalman_filter view_imu_kalman_filter.launch.py
```
