#include "imu_kalman_filter/IMUKalmanFilterPublisher.hpp"

using namespace std::chrono_literals;

KalmanFilter * kf;

IMUKalmanFilterPublisher::IMUKalmanFilterPublisher() : Node("imu_kalman_filter_publisher"),
    linear_x_(RollingWindow::window_size = 5),
    linear_y_(RollingWindow::window_size = 5),
    linear_z_(RollingWindow::window_size = 5)
{
    RCLCPP_INFO(this->get_logger(), "[IMUKalmanFilterPublisher::IMUKalmanFilterPublisher] Run node.");
    
    this->declare_parameter("rolling_mean", true);
    rolling_mean_ = this->get_parameter("rolling_mean").get_value<bool>();

    this->declare_parameter("rolling_window_size", 5);
    rolling_window_size_ = this->get_parameter("rolling_window_size").get_value<size_t>();

    if (rolling_mean_)
        resetAccumulators();    
    
    const auto QOS_RKL10V = rclcpp::QoS(rclcpp::KeepLast(10)).reliable().durability_volatile();

    imu_sub_ = this->create_subscription<Imu>("imu/data_raw", QOS_RKL10V, std::bind(
                &IMUKalmanFilterPublisher::imu_callback, this, std::placeholders::_1));
    mag_sub_ = this->create_subscription<MagneticField>("mag/data_raw", QOS_RKL10V, std::bind(
                &IMUKalmanFilterPublisher::mag_callback, this, std::placeholders::_1));

    imu_pub_ = this->create_publisher<Imu>("imu/data", QOS_RKL10V);
    timer_ = this->create_wall_timer(1ms, std::bind(&IMUKalmanFilterPublisher::timer_callback, this));

    gyro_.p = 0.0;
    gyro_.q = 0.0;
    gyro_.r = 0.0;

    acc_.x = 0.0;
    acc_.y = 0.0;
    acc_.z = 0.0;

    mag_.x = 0.0;
    mag_.y = 0.0;
    mag_.z = 0.0;

    IMU_READ_ = false;
    MAG_READ_ = false;
}

IMUKalmanFilterPublisher::~IMUKalmanFilterPublisher()
{
    delete kf;
}

void IMUKalmanFilterPublisher::resetAccumulators()
{
    linear_x_ = RollingMean(RollingWindow::window_size = rolling_window_size_);
    linear_y_ = RollingMean(RollingWindow::window_size = rolling_window_size_);
    linear_z_ = RollingMean(RollingWindow::window_size = rolling_window_size_);
}

void IMUKalmanFilterPublisher::imu_callback(const Imu::SharedPtr msg)
{
    gyro_.p = msg->angular_velocity.x;
    gyro_.q = msg->angular_velocity.y;
    gyro_.r = msg->angular_velocity.z;

    if (rolling_mean_){
        linear_x_(msg->linear_acceleration.x);
        linear_y_(msg->linear_acceleration.y);
        linear_z_(msg->linear_acceleration.z);

        acc_.x = rolling_mean(linear_x_);
        acc_.y = rolling_mean(linear_y_);
        acc_.z = rolling_mean(linear_z_);
    }
    else{
        acc_.x = msg->linear_acceleration.x;
        acc_.y = msg->linear_acceleration.y;
        acc_.z = msg->linear_acceleration.z;
    }

    IMU_READ_ = true;
}

void IMUKalmanFilterPublisher::mag_callback(const MagneticField::SharedPtr msg)
{
    mag_.x = msg->magnetic_field.x;
    mag_.y = msg->magnetic_field.y;
    mag_.z = msg->magnetic_field.z;

    MAG_READ_ = true;
}

void IMUKalmanFilterPublisher::timer_callback()
{
    if (!IMU_READ_ || !MAG_READ_)
        return;

    EulerAngle euler;
    Quaternion quater;
    Quaternion filtered_quater;

    Acceleration2EulerAngle(acc_, &euler);
    Magnetic2EulerAngle(mag_, &euler);
    EulerAngle2Quaternion(euler, &quater);

    kf->step(gyro_, quater, &filtered_quater);

    Imu msg;

    msg.linear_acceleration.x = acc_.x;
    msg.linear_acceleration.y = acc_.y;
    msg.linear_acceleration.z = acc_.z;

    msg.angular_velocity.x = gyro_.p;
    msg.angular_velocity.y = gyro_.q;
    msg.angular_velocity.z = gyro_.r;

    msg.orientation.w = filtered_quater.w;
    msg.orientation.x = filtered_quater.x;
    msg.orientation.y = filtered_quater.y;
    msg.orientation.z = filtered_quater.z;
    
    imu_pub_->publish(msg);

    IMU_READ_ = false;
    MAG_READ_ = false;    
}

void IMUKalmanFilterPublisher::Acceleration2EulerAngle(Acceleration acc, EulerAngle * euler)
{
    euler->pitch = std::atan2(-acc.x, std::sqrt(acc.y * acc.y + acc.z * acc.z));
    euler->roll = std::atan2(acc.y, acc.z);
}

void IMUKalmanFilterPublisher::Magnetic2EulerAngle(Magnetic mag, EulerAngle * euler)
{
    double cr = std::cos(euler->roll);
    double sr = std::sin(euler->roll);
    double cp = std::cos(euler->pitch);
    double sp = std::sin(euler->pitch);

    euler->yaw = std::atan2(-(mag.x * sr * sp + mag.y * cr - mag.z * sr * cp),
                             (mag.x * cp + mag.z * sp));
}

void IMUKalmanFilterPublisher::EulerAngle2Quaternion(EulerAngle euler, Quaternion * q)
{
    double cr = std::cos(euler.roll * 0.5);
    double sr = std::sin(euler.roll * 0.5);
    double cp = std::cos(euler.pitch * 0.5);
    double sp = std::sin(euler.pitch * 0.5);
    double cy = std::cos(euler.yaw * 0.5);
    double sy = std::sin(euler.yaw * 0.5);

    q->w = cr * cp * cy + sr * sp * sy;
    q->x = sr * cp * cy - cr * sp * sy;
    q->y = cr * sp * cy + sr * cp * sy;
    q->z = cr * cp * sy - sr * sp * cy;
}

int main(int argc, char * argv[])
{
    kf = KalmanFilter::getKalmanFilter();

    rclcpp::init(argc, argv);
    auto imu_kalman_filter_publisher = std::make_shared<IMUKalmanFilterPublisher>();
    rclcpp::spin(imu_kalman_filter_publisher);
    rclcpp::shutdown();

    return 0;
}
