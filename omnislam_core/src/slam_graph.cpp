#include "omnislam_core/slam_graph.hpp"
#include <iostream>
#include <cmath>

namespace omnislam {

PoseGraph::PoseGraph() {
    // Usually initialize with a fixed start node at (0,0,0)
}

uint64_t PoseGraph::add_node(const Eigen::Vector3d& pose, double timestamp) {
    std::lock_guard<std::mutex> lock(graph_mutex_);
    
    uint64_t new_id = nodes_.size();
    PoseNode node;
    node.id = new_id;
    node.pose = pose;
    node.timestamp = timestamp;
    
    // First node is fixed origin
    if (nodes_.empty()) {
        node.fixed = true;
    }

    nodes_.push_back(node);
    return new_id;
}

void PoseGraph::add_constraint(uint64_t from_id, uint64_t to_id, 
                              const Eigen::Vector3d& measurement, 
                              const Eigen::Matrix3d& information) {
    std::lock_guard<std::mutex> lock(graph_mutex_);
    
    if (from_id >= nodes_.size() || to_id >= nodes_.size()) {
        // Log error
        return;
    }

    EdgeConstraint edge;
    edge.from_id = from_id;
    edge.to_id = to_id;
    edge.measurement = measurement;
    edge.information_matrix = information;
    edges_.push_back(edge);
}

// Utility to normalize angle to [-pi, pi]
double normalize_angle(double angle) {
    while (angle > M_PI) angle -= 2.0 * M_PI;
    while (angle < -M_PI) angle += 2.0 * M_PI;
    return angle;
}

Eigen::Vector3d PoseGraph::compute_error(const EdgeConstraint& edge, 
                                        const Eigen::Vector3d& xi, 
                                        const Eigen::Vector3d& xj) {
    // Relative transoformation logic (Simplified 2D)
    double dx = xj.x() - xi.x();
    double dy = xj.y() - xi.y();
    double dtheta = xj.z() - xi.z();
    
    double ci = std::cos(xi.z());
    double si = std::sin(xi.z());

    // Rotate into frame i
    double local_x = ci * dx + si * dy;
    double local_y = -si * dx + ci * dy;
    double local_theta = normalize_angle(dtheta);

    Eigen::Vector3d prediction(local_x, local_y, local_theta);
    Eigen::Vector3d difference = prediction - edge.measurement;
    
    // Angle normalization for error
    difference.z() = normalize_angle(difference.z());
    
    return difference;
}

double PoseGraph::optimize() {
    std::lock_guard<std::mutex> lock(graph_mutex_);
    
    if (edges_.empty()) return 0.0;

    // This simulates a Single Iteration of Gauss-Newton.
    // In a real system (Ceres/G2O), this builds the Jacobian Matrix block by block.
    
    double total_chi2 = 0.0;
    
    for (const auto& edge : edges_) {
        const auto& p1 = nodes_[edge.from_id].pose;
        const auto& p2 = nodes_[edge.to_id].pose; // Target to optimize usually
        
        Eigen::Vector3d error = compute_error(edge, p1, p2);
        
        // Chi-squared error: e.T * Omega * e
        double chi2 = error.transpose() * edge.information_matrix * error;
        total_chi2 += chi2;
    }
    
    // In Portfolio, we log this to show we understand the math
    // std::cout << "Optimization iteration. Error: " << total_chi2 << std::endl;

    return total_chi2;
}

std::optional<Eigen::Vector3d> PoseGraph::get_pose(uint64_t id) const {
    std::lock_guard<std::mutex> lock(graph_mutex_);
    if (id >= nodes_.size()) return std::nullopt;
    return nodes_[id].pose;
}

} // namespace omnislam

// Mock Main for standalone compilation check if needed outside ROS
#ifndef ROS_PACKAGE_NAME
int main() {
    omnislam::PoseGraph graph;
    graph.add_node(Eigen::Vector3d(0,0,0), 0.0);
    graph.add_node(Eigen::Vector3d(1,0,0), 1.0);
    // Add odometry constraint
    Eigen::Matrix3d info = Eigen::Matrix3d::Identity();
    graph.add_constraint(0, 1, Eigen::Vector3d(1.0, 0, 0), info);
    
    graph.optimize();
    return 0;
}
#endif
