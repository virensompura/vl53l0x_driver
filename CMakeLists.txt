cmake_minimum_required(VERSION 3.8)
project(vl53l0x_driver)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(rclcpp_lifecycle REQUIRED)

add_library(${PROJECT_NAME}
  src/vl53l0x.cpp
  src/vl53l0x_publisher.cpp
)

target_include_directories(${PROJECT_NAME} PRIVATE include)

ament_target_dependencies(${PROJECT_NAME}
  rclcpp
  sensor_msgs
  rclcpp_lifecycle
)

add_executable(vl53l0x_driver_node 
  src/vl53l0x_publisher.cpp
  src/vl53l0x.cpp
)

ament_target_dependencies(vl53l0x_driver_node
  rclcpp
  sensor_msgs
  rclcpp_lifecycle
)

install(TARGETS ${PROJECT_NAME} vl53l0x_driver_node
  DESTINATION lib/${PROJECT_NAME}
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()
endif()

install(
  DIRECTORY include launch src
  DESTINATION lib/${PROJECT_NAME}
)

ament_package()
