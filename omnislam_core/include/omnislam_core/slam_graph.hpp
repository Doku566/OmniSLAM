#pragma once

#include <vector>
#include <mutex>
#include <Eigen/Dense>
#include <memory>
#include <optional>

namespace omnislam {

/**
 * @brief Represents a robot state (x, y, theta) at a specific timestamp.
 */
struct PoseNode {
    uint64_t id;
    double timestamp;
    Eigen::Vector3d pose; // [x, y, theta]
    
    // Fixed poses (like the start) shouldn't be moved by optimizer
    bool fixed = false;
};

/**
 * @brief Represents a constraint between two nodes.
 * Could be odometry (sequential) or loop closure (non-sequential).
 */
struct EdgeConstraint {
    uint64_t from_id;
    uint64_t to_id;
    Eigen::Vector3d measurement; // Relative transformation
    Eigen::Matrix3d information_matrix; // Inverse covariance
};

/**
 * @brief Thread-safe Pose Graph Optimizer.
 * 
 * Manages the "backend" of the SLAM system. It accepts new poses and constraints
 * and solves the non-linear least squares problem to correct trajectory drift.
 */
class PoseGraph {
public:
    PoseGraph();
    
    /**
     * @brief Add a new robot pose to the graph.
     * @param pose Estimated global pose [x, y, theta]
     * @return Node ID
     */
    uint64_t add_node(const Eigen::Vector3d& pose, double timestamp);

    /**
     * @brief Add a constraint (Edge) between two nodes.
     */
    void add_constraint(uint64_t from_id, uint64_t to_id, 
                       const Eigen::Vector3d& measurement, 
                       const Eigen::Matrix3d& information);

    /**
     * @brief Trigger the optimization process.
     * Uses Gauss-Newton or Levenberg-Marquardt to minimize error.
     * @return Final error metric
     */
    double optimize();

    /**
     * @brief Get current estimate of a specific node.
     */
    std::optional<Eigen::Vector3d> get_pose(uint64_t id) const;

    size_t node_count() const { return nodes_.size(); }

private:
    // Thread safety for async optimization loop vs real-time tracking
    mutable std::mutex graph_mutex_;

    std::vector<PoseNode> nodes_;
    std::vector<EdgeConstraint> edges_;

    // Error function for a single edge
    Eigen::Vector3d compute_error(const EdgeConstraint& edge, 
                                 const Eigen::Vector3d& p1, 
                                 const Eigen::Vector3d& p2);
};

} // namespace omnislam
