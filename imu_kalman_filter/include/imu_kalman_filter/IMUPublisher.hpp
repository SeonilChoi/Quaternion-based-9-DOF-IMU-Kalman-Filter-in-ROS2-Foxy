#ifndef IMU_PUBLISHER_HPP_
#define IMU_PUBLISHER_HPP_

#include <rclcpp/rclcpp.hpp>

#include "sensor_msgs/msg/imu.hpp"
#include "sensor_msgs/msg/magnetic_field.hpp"

class IMUPublisher : public rclcpp::Node
{
public:
    using Imu = sensor_msgs::msg::Imu;
    using MagneticField = sensor_msgs::msg::MagneticField;

    IMUPublisher();
    virtual ~IMUPublisher();

private:
    rclcpp::Publisher<Imu>::SharedPtr imu_pub_;
    rclcpp::Publisher<MagneticField>::SharedPtr mag_pub_;
    rclcpp::TimerBase::SharedPtr timer_;

    void timer_callback();

    void makeImuMsg(uint8_t * raw, Imu * imu_msg, MagneticField *  mag_msg);
};

#endif
