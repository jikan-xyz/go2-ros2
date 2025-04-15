cmake_minimum_required(VERSION 3.8)
project(go2_basics)

# Compiler options
if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# Directories
include_directories(include include/common)
link_directories(src)

# Find dependencies
find_package(ament_cmake REQUIRED)
find_package(unitree_go REQUIRED)
find_package(unitree_api REQUIRED)
find_package(rclcpp REQUIRED)

# List of dependencies
set(
  DEPENDENCY_LIST
  unitree_go
  unitree_api
  rclcpp
)

# Add executables
add_executable(stand_up src/stand_up.cpp src/common/ros2_sport_client.cpp)
add_executable(sit src/sit.cpp src/common/ros2_sport_client.cpp)
add_executable(stand_down src/stand_down.cpp src/common/ros2_sport_client.cpp)
add_executable(lock src/lock_joints.cpp src/common/ros2_sport_client.cpp)
add_executable(unlock src/unlock_joints.cpp src/common/ros2_sport_client.cpp)
add_executable(hello src/hello.cpp src/common/ros2_sport_client.cpp)
add_executable(jump src/jump.cpp src/common/ros2_sport_client.cpp)
add_executable(pounce src/pounce.cpp src/common/ros2_sport_client.cpp)
add_executable(scrape src/scrape.cpp src/common/ros2_sport_client.cpp)
add_executable(lidar_on src/lidar_on.cpp)
add_executable(lidar_off src/lidar_off.cpp)

# Link dependencies to executables
ament_target_dependencies(stand_up ${DEPENDENCY_LIST})
ament_target_dependencies(sit ${DEPENDENCY_LIST})
ament_target_dependencies(stand_down ${DEPENDENCY_LIST})
ament_target_dependencies(lock ${DEPENDENCY_LIST})
ament_target_dependencies(unlock ${DEPENDENCY_LIST})
ament_target_dependencies(hello ${DEPENDENCY_LIST})
ament_target_dependencies(jump ${DEPENDENCY_LIST})
ament_target_dependencies(pounce ${DEPENDENCY_LIST})
ament_target_dependencies(scrape ${DEPENDENCY_LIST})
ament_target_dependencies(lidar_on ${DEPENDENCY_LIST})
ament_target_dependencies(lidar_off ${DEPENDENCY_LIST})

# Install executables
install(TARGETS stand_up DESTINATION lib/${PROJECT_NAME})
install(TARGETS sit DESTINATION lib/${PROJECT_NAME})
install(TARGETS stand_down DESTINATION lib/${PROJECT_NAME})
install(TARGETS lock DESTINATION lib/${PROJECT_NAME})
install(TARGETS unlock DESTINATION lib/${PROJECT_NAME})
install(TARGETS hello DESTINATION lib/${PROJECT_NAME})
install(TARGETS jump DESTINATION lib/${PROJECT_NAME})
install(TARGETS pounce DESTINATION lib/${PROJECT_NAME})
install(TARGETS scrape DESTINATION lib/${PROJECT_NAME})
install(TARGETS lidar_on DESTINATION lib/${PROJECT_NAME})
install(TARGETS lidar_off DESTINATION lib/${PROJECT_NAME})

# Testing dependencies
if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  set(ament_cmake_copyright_FOUND TRUE)
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
