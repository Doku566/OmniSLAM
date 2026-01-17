cmake_minimum_required(VERSION 3.8)
project(omnislam_core)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# ROS 2 Dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(Eigen3 REQUIRED)
# We assume Ceres is installed in the system/container
# find_package(Ceres REQUIRED) 

include_directories(include)

# Library: SLAM Core
add_library(slam_backend
  src/slam_graph.cpp
  src/ekf.cpp
)
target_include_directories(slam_backend PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
  ${EIGEN3_INCLUDE_DIR}
)
target_link_libraries(slam_backend
  ${Eigen3_LIBRARIES}
  # ${CERES_LIBRARIES} # Uncomment when real solver is present
)
ament_target_dependencies(slam_backend
  rclcpp
  geometry_msgs
)

# Executable: Main ROS Node (Placeholder for portfolio structure)
add_executable(slam_node src/slam_graph.cpp) 
# Note: In real life main() would be separate, but reusing file for compilation check
target_link_libraries(slam_node slam_backend)

# Install rules
install(
  DIRECTORY include/
  DESTINATION include
)
install(
  TARGETS slam_backend slam_node
  EXPORT export_slam_backend
  LIBRARY DESTINATION lib
  ARCHIVE DESTINATION lib
  RUNTIME DESTINATION lib/${PROJECT_NAME}
)

ament_export_include_directories(include)
ament_export_libraries(slam_backend)
ament_package()
