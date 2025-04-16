cmake_minimum_required(VERSION 3.8)
project(go2_localisation)

# Compiler options
if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# Directories
include_directories(include include/common)
link_directories(src)

# Dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(pcl_conversions REQUIRED)
find_package(pcl_ros REQUIRED)
find_package(std_msgs REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(unitree_go REQUIRED)
find_package(PCL REQUIRED COMPONENTS common io)

# Regroup les deps ici pour simplifier la suite
set(DEPENDENCY_LIST
  rclcpp
  sensor_msgs
  pcl_conversions
  pcl_ros
  std_msgs
  tf2_ros
  geometry_msgs
  unitree_go
)

# Executables
add_executable(lidar_fixer src/lidar_fixer.cpp)
add_executable(orientation src/orientation_tf2.cpp)

target_link_libraries(lidar_fixer ${PCL_LIBRARIES})

ament_target_dependencies(lidar_fixer ${DEPENDENCY_LIST})
ament_target_dependencies(orientation ${DEPENDENCY_LIST})

# Install
install(TARGETS lidar_fixer DESTINATION lib/${PROJECT_NAME})
install(TARGETS orientation DESTINATION lib/${PROJECT_NAME})

# Testing
if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  set(ament_cmake_copyright_FOUND TRUE)
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
