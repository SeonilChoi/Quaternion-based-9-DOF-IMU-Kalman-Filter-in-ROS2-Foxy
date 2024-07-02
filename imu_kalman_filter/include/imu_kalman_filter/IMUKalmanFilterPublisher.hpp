#ifndef IMU_KALMAN_FILTER_PUBLISHER_HPP
#define IMU_KALMAN_FILTER_PUBLISHER_HPP

#include <boost/accumulators/accumulators.hpp>
#include <boost/accumulators/statistics/stats.hpp>
#include <boost/accumulators/statistics/rolling_mean.hpp>

#include "imu_kalman_filter/KalmanFilter.hpp"

#include <rclcpp/rclcpp.hpp>

#include "sensor_msgs/msg/imu.hpp"
#include "sensor_msgs/msg/magnetic_field.hpp"

using namespace boost::accumulators;

class IMUKalmanFilterPublisher : public rclcpp::Node
{
public:
    using Imu = sensor_msgs::msg::Imu;
    using MagneticField = sensor_msgs::msg::MagneticField;

    IMUKalmanFilterPublisher();
    virtual ~IMUKalmanFilterPublisher();

    Gyroscope gyro_;
    Acceleration acc_;
    Magnetic mag_;
    EulerAngle euler_;

private:
    rclcpp::Subscription<Imu>::SharedPtr imu_sub_;
    rclcpp::Subscription<MagneticField>::SharedPtr mag_sub_;

    rclcpp::Publisher<Imu>::SharedPtr imu_pub_;
    rclcpp::TimerBase::SharedPtr timer_;

    void imu_callback(const Imu::SharedPtr msg);
    void mag_callback(const MagneticField::SharedPtr msg);
    void timer_callback();

    void Acceleration2EulerAngle(Acceleration acc, EulerAngle * euler);
    void Magnetic2EulerAngle(Magnetic mag, EulerAngle * euler);
    void EulerAngle2Quaternion(EulerAngle euler, Quaternion * q);

    bool IMU_READ_;
    bool MAG_READ_;

    typedef accumulator_set<double, stats<tag::rolling_mean>> RollingMean;
    typedef tag::rolling_window RollingWindow;

    void resetAccumulators();

    bool rolling_mean_;
    size_t rolling_window_size_;
    RollingMean linear_x_;
    RollingMean linear_y_;
    RollingMean linear_z_;
};

#endif
