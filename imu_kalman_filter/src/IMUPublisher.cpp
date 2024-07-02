#include <chrono>
#include <cmath>

#include "imu_kalman_filter/IMUPublisher.hpp"
#include "imu_kalman_filter/PortHandler.hpp"

#define BAUDRATE  38400
#define IMU_DEVICE "/dev/ttyACM1"

using namespace std::chrono_literals;

PortHandler * imu_port;

IMUPublisher::IMUPublisher() : Node("imu_publisher")
{
    RCLCPP_INFO(this->get_logger(), "[IMUPublisher::IMUPublisher] Run node.");

    this->declare_parameter("MPU6050", true);
    MPU6050_ = this->get_parameter("MPU6050").get_value<bool>();

    this->declare_parameter("AK8963", true);
    AK8963_ = this->get_parameter("AK8963").get_value<bool>();

    if (MPU6050_ && AK8963_){
        write_length_ = 4;
        write_data_ = new uint8_t[write_length_]{3, 59, 67, 3};
    }
    else if (MPU6050_){
        write_length_ = 3;
        write_data_ = new uint8_t[write_length_]{2, 59, 67};
    }
    else if (AK8963_){
        write_length_ = 2;
        write_data_ = new uint8_t[write_length_]{1, 3};
    }
    else{
        RCLCPP_INFO(this->get_logger(), "[IMUPublisher::IMUPublisher] No read data.");
        return;
    }

    if (write_data_ == nullptr)
        return;
    
    const auto QOS_RKL10V = rclcpp::QoS(rclcpp::KeepLast(10)).reliable().durability_volatile();

    imu_pub_ = this->create_publisher<Imu>("imu/data_raw", QOS_RKL10V);
    mag_pub_ = this->create_publisher<MagneticField>("mag/data_raw", QOS_RKL10V);
    timer_ = this->create_wall_timer(1ms, std::bind(&IMUPublisher::timer_callback, this));
}

IMUPublisher::~IMUPublisher()
{
    delete imu_port;
    delete[] write_data_;
}

void IMUPublisher::timer_callback()
{   
    int result = imu_port->writePort(write_data_, write_length_);
    
    if (result != COMM_SUCCESS)
        return;

    uint8_t read_length = (write_length_ - 1) * 6;
    uint8_t * read_data = new uint8_t[read_length];
    if (read_data == nullptr)
        return;

    result = imu_port->readPort(read_data, read_length);
    if (result != COMM_SUCCESS){
        delete[] read_data;
        return;    
    }

    int16_t * imu_data = new int16_t[(write_length_ - 1) * 3];
    if (imu_data == nullptr)
        return;
    
    convertRaw2Imu(read_data, imu_data);
    if (MPU6050_ && AK8963_){
        convertData2ImuMsg(imu_data);
        convertData2MagMsg(imu_data + 6);
    }
    else if(MPU6050_){
        convertData2ImuMsg(imu_data);
    }
    else if(AK8963_){
        convertData2MagMsg(imu_data);
    }
    
    delete[] read_data;
    delete[] imu_data;
}

void IMUPublisher::convertRaw2Imu(uint8_t * raw, int16_t * data)
{
    for (uint8_t s = 0; s < (write_length_ - 1) * 3; s++)
    {
        data[s] = (static_cast<int16_t>(raw[s*2]) << 8) | raw[s*2+1];
    }
}

void IMUPublisher::convertData2ImuMsg(int16_t * data)
{
    Imu imu_msg;
    
    imu_msg.linear_acceleration.x = *(data + 0) / 16384.0 * 9.80665;
    imu_msg.linear_acceleration.y = *(data + 1) / 16384.0 * 9.80665;
    imu_msg.linear_acceleration.z = *(data + 2) / 16384.0 * 9.80665;

    imu_msg.angular_velocity.x = *(data + 3) / 131.0 / 180.0 * M_PI;
    imu_msg.angular_velocity.y = *(data + 4) / 131.0 / 180.0 * M_PI;
    imu_msg.angular_velocity.z = *(data + 5) / 131.0 / 180.0 * M_PI;
    
    imu_msg.header.stamp = this->get_clock()->now();
    
    imu_pub_->publish(imu_msg);
}

void IMUPublisher::convertData2MagMsg(int16_t * data)
{
    MagneticField mag_msg;
    
    mag_msg.magnetic_field.x = *(data + 0) * 1200.0 / 4096.0;
    mag_msg.magnetic_field.y = *(data + 1) * 1200.0 / 4096.0;
    mag_msg.magnetic_field.z = *(data + 2) * 1200.0 / 4096.0;
    
    mag_pub_->publish(mag_msg);
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
