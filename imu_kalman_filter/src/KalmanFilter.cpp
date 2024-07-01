#include "imu_kalman_filter/KalmanFilter.hpp"

#include <cmath>

KalmanFilter * KalmanFilter::getKalmanFilter()
{
    return (KalmanFilter *) (new KalmanFilter());
}

KalmanFilter::KalmanFilter()
{
    dt_ = 0.01;

    A_ = Eigen::Matrix<double, VEC_SIZE, VEC_SIZE>::Identity();
    P_ = Eigen::Matrix<double, VEC_SIZE, VEC_SIZE>::Identity();
    H_ = Eigen::Matrix<double, VEC_SIZE, VEC_SIZE>::Identity();
    Q_ = Eigen::Matrix<double, VEC_SIZE, VEC_SIZE>::Identity() * 0.0001;
    R_ = Eigen::Matrix<double, VEC_SIZE, VEC_SIZE>::Identity() * 0.0001;
    K_ = Eigen::Matrix<double, VEC_SIZE, VEC_SIZE>::Zero();
    x_ << 1.0, 0.0, 0.0, 0.0;
    z_ = Eigen::VectorXd::Zero(VEC_SIZE);
}

KalmanFilter::~KalmanFilter()
{
    x_.resize(0);
    z_.resize(0);
}

void KalmanFilter::step(Gyroscope gyro, Quaternion sensor, Quaternion * quater)
{
    setTransitionMatrix(gyro);

    predict();
    compute();

    getStateVector(quater);
    normalizeQuaternion(quater);
    update(&x_, (*quater));

    normalizeQuaternion(&sensor);
    update(&z_, sensor);
    
    correction();
    getStateVector(quater);
    normalizeQuaternion(quater);
    update(&x_, (*quater)); 
}

void KalmanFilter::setTransitionMatrix(Gyroscope gyro)
{
    Eigen::Matrix<double, VEC_SIZE, VEC_SIZE> T;
    T << 0.0,    -gyro.p, -gyro.q, -gyro.r,
         gyro.p,  0.0,     gyro.r, -gyro.q,
         gyro.q, -gyro.r,  0.0,     gyro.p,
         gyro.r,  gyro.q, -gyro.p,  0.0;

    A_ = Eigen::Matrix<double, VEC_SIZE, VEC_SIZE>::Identity() + dt_ * T;
}

void KalmanFilter::getStateVector(Quaternion * q)
{
    q->w = x_(0);
    q->x = x_(1);
    q->y = x_(2);
    q->z = x_(3);
}

void KalmanFilter::normalizeQuaternion(Quaternion * q)
{
    double norm = std::sqrt(q->w * q->w + q->x * q->x + q->y * q->y + q->z * q->z);

    q->w /= norm;
    q->x /= norm;
    q->y /= norm;
    q->z /= norm;
}

void KalmanFilter::predict()
{
    x_ = A_ * x_;
    P_ = A_ * P_ * A_.transpose() + Q_;
}

void KalmanFilter::compute()
{
    K_ = P_ * H_.transpose() * (H_ * P_ * H_.transpose() + R_).inverse();
}

void KalmanFilter::update(Eigen::VectorXd * vec, Quaternion q)
{
    (* vec) << q.w, q.x, q.y, q.z;
}

void KalmanFilter::correction()
{
    x_ = x_ + K_ * (z_ - H_ * x_);
    P_ = P_ - K_ * H_ * P_;
}
