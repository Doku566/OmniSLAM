# OmniSLAM: Graph-Based Navigation for Differential Drive Robots

`OmniSLAM` provides a custom C++ implementation of the SLAM backend and State Estimation filters, integrated into a standard ROS 2 Humble structure. This project manually implements the core probability math often abstracted away by libraries like `robot_localization` or `gmapping`.

## Design Decisions

### 1. Manual EKF Derivation
Instead of using a generic Kalman Filter library, I derived and implemented the **Extended Kalman Filter (EKF)** for a Differential Drive kinematic model.
-   **Why**: To explicitly handle the non-linearities of the motion model ($x' = x + v \cos{\theta} dt$). Standard Linear Kalman Filters diverge immediately in robotics contexts.
-   **Jacobians**: The state transition Jacobian $G_t$ is computed analytically in `ekf.cpp` to linearize the uncertainty propagation around the current operating point.

### 2. Graph-Based Optimization (Backend)
The system uses a Pose Graph architecture rather than a Particle Filter (MCL) for the mapping backend.
-   **Why**: Particle filters struggle with loop closure in large environments due to particle depletion. Graph optimization (Least Squares on constraints) allows correcting the entire trajectory history when a loop is closed.

## Trade-offs and Limitations

*   **Linearization Error**: The EKF relies on a first-order Taylor expansion. In highly dynamic turns with large time steps ($dt$), the linearization error accumulates, causing the estimated covariance $P$ to become inconsistent with true error. An Unscented Kalman Filter (UKF) would improve this but is computationally heavier.
*   **Sensor Noise Models**: The current `Q` (Process Noise) and `R` (Measurement Noise) matrices are tuned with fixed constants. In a production system, these should ideally be adaptive.
*   **2D Only**: The system assumes a planar world (`z=0`). It will fail on ramps or uneven terrain.

## Current Status

-   [x] **State Estimation**: Manual EKF implemented (`src/ekf.cpp`) with Jacobian derivation.
-   [x] **Graph Backend**: `PoseGraph` structure and error functions implemented.
-   [x] **Ros Wrapper**: `package.xml` and `CMakeLists.txt` fully compliant with ROS 2 Humble.
-   [ ] **Solver Integration**: The optimization loop (`optimize()`) computes the Chi-Square error but currently mocks the actual solver step (Placeholder for Ceres/G2O integration).

## Complexity Analysis

### EKF Prediction Step
*   **Time**: $O(D^3)$ due to matrix multiplication of covariance $P$ ($3 \times 3$), which effectively is constant $O(1)$ for this fixed state size.

### Graph Optimization (One Iteration)
*   **Time**: $O(N \cdot k^2)$ per iteration using Sparse Cholesky, where $N$ is the number of poses and $k$ is the average degree (connectivity).
*   **Space**: $O(N)$ to store the trajectory.
