#ifndef IMU_PUBLISHER_HPP_
#define IMU_PUBLISHER_HPP_

#include <rclcpp/rclcpp.hpp>

#include "sensor_msgs/msg/imu.hpp"

class IMUPublisher : public rclcpp::Node
{
public:
    using Imu = sensor_msgs::msg::Imu;

    IMUPublisher();
    virtual ~IMUPublisher();

private:
    rclcpp::Publisher<Imu>::SharedPtr pub_;
    rclcpp::TimerBase::SharedPtr timer_;

    void timer_callback();

    sensor_msgs::msg::Imu makeImuMsg(uint8_t * raw);
};

#endif
