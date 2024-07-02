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

    void convertRaw2Imu(uint8_t * raw, int16_t * data);
    void convertData2ImuMsg(int16_t * data);
    void convertData2MagMsg(int16_t * data);

    bool MPU6050_;
    bool AK8963_;

    uint8_t * write_data_;
    uint8_t write_length_;
};

#endif
