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
find_package(tf2_eigen REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
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
  tf2_eigen
  geometry_msgs
  nav_msgs
  unitree_go
  PCL
)

# Executables
add_executable(lidar_fixer src/lidar_fixer.cpp)
add_executable(tf_odom src/tf_odom.cpp)

# Lier avec les bibliothèques PCL
target_link_libraries(lidar_fixer ${PCL_LIBRARIES})

# Ajouter les dépendances ROS2 aux cibles
ament_target_dependencies(lidar_fixer ${DEPENDENCY_LIST})
ament_target_dependencies(tf_odom ${DEPENDENCY_LIST})

# Install
install(TARGETS lidar_fixer DESTINATION lib/${PROJECT_NAME})
install(TARGETS tf_odom DESTINATION lib/${PROJECT_NAME})

# Testing
if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  set(ament_cmake_copyright_FOUND TRUE)
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()