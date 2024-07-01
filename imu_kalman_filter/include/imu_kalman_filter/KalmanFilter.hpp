#ifndef KALMAN_FILTER_HPP_
#define KALMAN_FILTER_HPP_

#include <Eigen/Dense>
#include "imu_kalman_filter/KalmanFilterUtils.hpp"

#define VEC_SIZE 4

class KalmanFilter
{
public:
    static KalmanFilter * getKalmanFilter();

    KalmanFilter();
    virtual ~KalmanFilter();

    void step(Gyroscope gyro, Quaternion sensor, Quaternion * quater);

private:
    void setTransitionMatrix(Gyroscope gyro);
    void getStateVector(Quaternion * q);
    void normalizeQuaternion (Quaternion *q);

    void predict();
    void compute();
    void update(Eigen::VectorXd * vec, Quaternion q);
    void correction();

    double dt_;

    Eigen::Matrix<double, VEC_SIZE, VEC_SIZE> A_;
    Eigen::Matrix<double, VEC_SIZE, VEC_SIZE> P_;
    Eigen::Matrix<double, VEC_SIZE, VEC_SIZE> H_;
    Eigen::Matrix<double, VEC_SIZE, VEC_SIZE> Q_;
    Eigen::Matrix<double, VEC_SIZE, VEC_SIZE> R_;
    Eigen::Matrix<double, VEC_SIZE, VEC_SIZE> K_;
    Eigen::VectorXd x_{VEC_SIZE};
    Eigen::VectorXd z_{VEC_SIZE};
};

#endif
