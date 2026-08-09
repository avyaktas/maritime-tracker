#pragma once
#include "types.hpp"
#include <Eigen/Dense>

class KalmanFilter {
public: 
    //process_noise: how much we distrust our constant velocity model
    //measurement_noise: std dev of GPS, in m
    KalmanFilter(double process_noise, double measurement_noise);

    //Project the stat forward dt seconds using the motion model
    void predict(double dt);

    //correct the state using GPS obs
    void update(const GpsMeasurement& z);

    const Eigen::Vector4d& state() const { return x_; }
    const Eigen::Matrix4d& covariance() const { return P_; }

    //measurment - prediciton
    const Eigen::Vector2d& innovation() const { return innovation_; }

private:
    Eigen::Vector4d x_;     //[px, py, vx, vy] (state, 2 position 2 velcoity)
    Eigen::Matrix4d P_;     // uncertaintiy in x_ (covariance matrix)
    Eigen::Matrix4d Q_;     // process noise
    Eigen::Matrix2d R_;     // measurment noise
    Eigen::Matrix<double,2,4> H_;   // maps state -> measuement
    Eigen::Vector2d innovation_ = Eigen::Vector2d::Zero();

    double q_;      //process noise scale, kept for rebuilidng Q with dt
};