#ifndef KALMAN_FILTER_UTILS_HPP_
#define KALMAN_FILTER_UTILS_HPP_

struct Gyroscope
{
    double p, q, r;
};

struct Acceleration
{
    double x, y, z;
};

struct Magnetic
{
    double x, y, z;
};

struct EulerAngle
{
    double roll, pitch, yaw;
};

struct Quaternion
{
    double w, x, y, z;
};

#endif
