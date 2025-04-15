cmake_minimum_required(VERSION 3.8)
project(go2_control)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

include_directories(include include/common)
link_directories(src)

# Find dependencies
find_package(ament_cmake REQUIRED)
find_package(unitree_go REQUIRED)
find_package(unitree_api REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)

# Lier les dépendances
set (
  DEPENDENCY_LIST
  unitree_go
  unitree_api
  rclcpp
  sensor_msgs
  geometry_msgs
)

# Ajout de l'exécutable
add_executable(teleop src/teleop.cpp src/common/ros2_sport_client.cpp)
add_executable(move src/move.cpp src/common/ros2_sport_client.cpp)

# Lier les dépendances à l'exécutable
ament_target_dependencies(teleop ${DEPENDENCY_LIST})
ament_target_dependencies(move ${DEPENDENCY_LIST})

# Installation de l'exécutable
install(TARGETS teleop DESTINATION lib/${PROJECT_NAME})
install(TARGETS move DESTINATION lib/${PROJECT_NAME})

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  # the following line skips the linter which checks for copyrights
  # comment the line when a copyright and license is added to all source files
  set(ament_cmake_copyright_FOUND TRUE)
  # the following line skips cpplint (only works in a git repo)
  # comment the line when this package is in a git repo and when
  # a copyright and license is added to all source files
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
