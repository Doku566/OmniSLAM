#include "omnislam_core/ekf.hpp"
#include <cmath>

namespace omnislam {

DifferentialDriveEKF::DifferentialDriveEKF() {
    x_.setZero();
    P_.setIdentity();
    
    // Initialize standard process noise (tunable)
    Q_.setIdentity();
    Q_(0,0) = 0.01; // Uncertainty in X propagation
    Q_(1,1) = 0.01; // Uncertainty in Y propagation
    Q_(2,2) = 0.005; // Uncertainty in Theta propagation
}

void DifferentialDriveEKF::initialize(const Eigen::Vector3d& init_state, const Eigen::Matrix3d& init_cov) {
    x_ = init_state;
    P_ = init_cov;
}

void DifferentialDriveEKF::predict(double v, double omega, double dt) {
    // 1. Extract state
    double theta = x_(2);
    
    // 2. Non-linear state transition function g(x, u)
    // x' = x + v * cos(theta) * dt
    // y' = y + v * sin(theta) * dt
    // theta' = theta + omega * dt
    
    // Avoid division by zero if using exact arc model, but for small dt, Euler integration is fine
    // Or, for high-fidelity: v/w * (-sin(theta) + sin(theta + w*dt)) etc.
    // Portfolio decision: Use simple Euler for clarity of Jacobian derivation
    
    double new_x = x_(0) + v * std::cos(theta) * dt;
    double new_y = x_(1) + v * std::sin(theta) * dt;
    double new_theta = x_(2) + omega * dt;
    
    // Normalize new_theta
    while (new_theta > M_PI) new_theta -= 2.0 * M_PI;
    while (new_theta < -M_PI) new_theta += 2.0 * M_PI;

    // 3. Jacobian G (Partial derivative of g with respect to x)
    // dg_x/dtheta = -v * sin(theta) * dt
    // dg_y/dtheta = v * cos(theta) * dt
    Eigen::Matrix3d G;
    G.setIdentity();
    G(0, 2) = -v * std::sin(theta) * dt;
    G(1, 2) =  v * std::cos(theta) * dt;
    
    // 4. Update Covariance
    // P = G * P * G^T + Q
    P_ = G * P_ * G.transpose() + Q_;
    
    // 5. Update State
    x_ << new_x, new_y, new_theta;
}

void DifferentialDriveEKF::update(const Eigen::Vector3d& measurement, const Eigen::Matrix3d& R) {
    // 1. Measurement Model h(x)
    // Assuming we measure state directly (e.g. GPS + Compass) -> Linear H = Identity
    // If we measured range/bearing to landmarks, H would be non-linear Jacobian.
    Eigen::Matrix3d H = Eigen::Matrix3d::Identity();
    
    Eigen::Vector3d z = measurement;
    Eigen::Vector3d y = z - H * x_; // Innovation (y)
    
    // Angle wrap for Innovation theta
    while (y(2) > M_PI) y(2) -= 2.0 * M_PI;
    while (y(2) < -M_PI) y(2) += 2.0 * M_PI;
    
    // 2. Innovation Covariance S
    // S = H * P * H^T + R
    Eigen::Matrix3d S = H * P_ * H.transpose() + R;
    
    // 3. Kalman Gain K
    // K = P * H^T * S^-1
    Eigen::Matrix3d K = P_ * H.transpose() * S.inverse();
    
    // 4. Update State
    x_ = x_ + K * y;
    
    // 5. Update Covariance
    // P = (I - K * H) * P
    Eigen::Matrix3d I = Eigen::Matrix3d::Identity();
    P_ = (I - K * H) * P_;
}

} // namespace omnislam
