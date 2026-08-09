#include "kalman_filter.hpp"

KalmanFilter::KalmanFilter(double process_noise, double measurement_noise)
    : q_(process_noise) {
    x_ = Eigen::Vector4d::Zero();

    //start uncertain, dont know position or velocity
    P_ = Eigen::Matrix4d::Identity() * 500.0;
    R_ = Eigen::Matrix2d::Identity() * (measurement_noise * measurement_noise);
    H_ << 1, 0, 0, 0,
          0, 1, 0, 0;
        
    Q_ = Eigen::Matrix4d::Identity() * q_;
}

void KalmanFilter::predict(double dt) {
    Eigen::Matrix4d F = Eigen::Matrix4d::Identity();
    F(0, 2) = dt;   // px += vx * dt
    F(1, 3) = dt;   // py += vy * dt
    x_ = F * x_;
    P_ = F * P_ * F.transpose() + Q_;
}


void KalmanFilter::update(const GpsMeasurement& z) {
    Eigen::Vector2d meas(z.x, z.y);
    innovation_ = meas - H_ * x_;
    Eigen::Matrix2d S = H_ * P_ * H_.transpose() + R_;
    Eigen::Matrix<double,4,2> K = P_ * H_.transpose() * S.inverse();
    x_ = x_ + K * innovation_;
    P_ = (Eigen::Matrix4d::Identity() - K * H_) * P_;
}