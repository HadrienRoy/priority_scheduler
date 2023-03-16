cmake_minimum_required(VERSION 3.5)
project(docking_scheduler)

# Default to C++14
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 14)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)

find_package(geometry_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(docking_interfaces REQUIRED)

include_directories(include)

set(DEPENDENCIES
  "rclcpp"
  "geometry_msgs"
  "sensor_msgs"
  "nav_msgs"
  "docking_interfaces"
)

add_executable( scheduler src/scheduler.cpp src/queue_update_server.cpp)
ament_target_dependencies( scheduler ${DEPENDENCIES} )

install(TARGETS
  scheduler
  DESTINATION lib/${PROJECT_NAME}
)

ament_package()
