cmake_minimum_required(VERSION 3.8)
project(my_robot_localization)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)

add_executable(kalman_filter src/kalman_filter_node.cpp src/kalman_filter.cpp)
target_include_directories(kalman_filter PRIVATE include)
ament_target_dependencies(kalman_filter rclcpp nav_msgs sensor_msgs)

install(TARGETS
        kalman_filter
        DESTINATION lib/${PROJECT_NAME}
)

ament_package()
