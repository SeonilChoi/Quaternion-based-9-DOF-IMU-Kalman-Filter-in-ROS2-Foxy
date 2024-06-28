#include <chrono>
#include <cmath>

#include "imu_kalman_filter/IMUPublisher.hpp"
#include "imu_kalman_filter/PortHandler.hpp"

#define BAUDRATE  9600
#define IMU_DEVICE "/dev/ttyACM1"

using namespace std::chrono_literals;

PortHandler * imu_port;

IMUPublisher::IMUPublisher() : Node("imu_publisher")
{
    RCLCPP_INFO(this->get_logger(), "[IMUPublisher::IMUPublisher] Run node.");

    const auto QOS_RKL10V = rclcpp::QoS(rclcpp::KeepLast(10)).reliable().durability_volatile();

    pub_ = this->create_publisher<Imu>("/imu/data_raw", QOS_RKL10V);
    timer_ = this->create_wall_timer(1ms, std::bind(&IMUPublisher::timer_callback, this));
}

IMUPublisher::~IMUPublisher()
{
    delete imu_port;
}

void IMUPublisher::timer_callback()
{   
    uint8_t write_data[4] = {3, 59, 67, 3};
    uint8_t write_length = 4;
    int result = imu_port->writePort(write_data, write_length);
    
    if (result != COMM_SUCCESS)
        return;

    uint8_t read_data[18];
    uint8_t read_length = 18;
    result = imu_port->readPort(read_data, read_length);
    
    if (result != COMM_SUCCESS)
        return;

    pub_->publish(makeImuMsg(read_data));
}

sensor_msgs::msg::Imu IMUPublisher::makeImuMsg(uint8_t * raw)
{
    int16_t data[9];
    sensor_msgs::msg::Imu msg;
    
    for (uint8_t s = 0; s < 9; s++)
    {
        data[s] = (static_cast<int16_t>(raw[s*2]) << 8) | raw[s*2+1];
    }

    msg.linear_acceleration.x = data[0] / 16384.0 * 9.80665;
    msg.linear_acceleration.y = data[1] / 16384.0 * 9.80665;
    msg.linear_acceleration.z = data[2] / 16384.0 * 9.80665;

    msg.angular_velocity.x = data[3] / 131.0 / 180.0 * M_PI;
    msg.angular_velocity.y = data[4] / 131.0 / 180.0 * M_PI;
    msg.angular_velocity.z = data[5] / 131.0 / 180.0 * M_PI;
    
    msg.header.stamp = this->get_clock()->now();

    return msg;
}

int main(int argc, char * argv[])
{
    imu_port = PortHandler::getPortHandler(IMU_DEVICE);

    int result = imu_port->openPort(BAUDRATE);

    if (result == false)
    {    
        printf("Failed");   
        return -1;
    }
    rclcpp::init(argc, argv);
    auto imu_node = std::make_shared<IMUPublisher>();
    rclcpp::spin(imu_node);
    rclcpp::shutdown();

    return 0;
}
