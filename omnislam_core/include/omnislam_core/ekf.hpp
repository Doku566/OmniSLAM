#pragma once

#include <Eigen/Dense>
#include <iostream>

namespace omnislam {

/**
 * @brief Extended Kalman Filter for a Differential Drive Robot.
 * 
 * Logic:
 * State x = [x, y, theta]^T
 * Control u = [v, omega]^T (Linear vel, Angular vel)
 */
class DifferentialDriveEKF {
public:
    DifferentialDriveEKF();

    /**
     * @brief Initialize state and covariance.
     */
    void initialize(const Eigen::Vector3d& init_state, const Eigen::Matrix3d& init_cov);

    /**
     * @brief Prediction Step (Motion Model).
     * Propagates the state distribution using non-linear kinematics.
     * 
     * @param v Linear velocity (m/s)
     * @param omega Angular velocity (rad/s)
     * @param dt Time delta (s)
     */
    void predict(double v, double omega, double dt);

    /**
     * @brief Update Step (Measurement Model).
     * Fuses absolute position measurement (e.g., GPS or SLAM Loop Closure).
     * 
     * @param measurement Measured [x, y, theta]
     * @param R Measurement noise covariance
     */
    void update(const Eigen::Vector3d& measurement, const Eigen::Matrix3d& R);

    Eigen::Vector3d get_state() const { return x_; }
    Eigen::Matrix3d get_covariance() const { return P_; }

private:
    Eigen::Vector3d x_; // State vector
    Eigen::Matrix3d P_; // State covariance
    
    // Process noise covariance (Q)
    Eigen::Matrix3d Q_; 
};

} // namespace omnislam
