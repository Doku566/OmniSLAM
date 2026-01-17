# OmniSLAM: Manual Slam Backend Implementation

A C++ study on Graph-Based Simultaneous Localization and Mapping (SLAM). This project implements the math for state estimation and graph optimization from scratch, rather than configuring existing ROS nodes.

## Core Component: The Extended Kalman Filter
The heart of the state estimation is `src/ekf.cpp`. I manually implemented the EKF for a Differential Drive robot.

### Design Decision: Analytical Jacobians
I derived the Jacobian matrices for the prediction step explicitly.
*   **Motion Model**: $x' = x + v \cos(\theta) dt$
*   **Jacobian ($G_t$)**:
    ```cpp
    // From src/ekf.cpp
    G(0, 2) = -v * std::sin(theta) * dt;
    G(1, 2) =  v * std::cos(theta) * dt;
    ```
    I chose to calculate this analytically rather than using Automatic Differentiation (AutoDiff) to force a deeper understanding of how uncertainty ($P$) propagates through the kinematic chain.

## Technical Challenges
**Linearization Divergence**:
During testing, I observed that the filter drifts significantly during sharp rotations if the update rate drops below 10Hz.
*   **Root Cause**: The EKF relies on a first-order Taylor series approximation. When $\omega$ (angular velocity) is high and $dt$ increases, the linear approximation of $\sin(\theta)$ fails to capture the true robot path, causing the covariance ellipse to detach from reality.

## Known Limitations (Trade-offs)
1.  **2D Planar Assumption**: The state vector is fixed to $[x, y, \theta]$. The system cannot estimate roll/pitch and will fail if the robot climbs a ramp.
2.  **No Solver Backend**: The `PoseGraph` class constructs the error function (Constraints), but the non-linear least squares solver (G2O/Ceres) is currently mocked. It computes error but does not minimize it yet.
3.  **Static Noise Matrices**: $Q$ (Process Noise) is hardcoded. It does not adapt to the robot's velocity (e.g., slip increases at high speeds), which limits accuracy in dynamic maneuvers.
