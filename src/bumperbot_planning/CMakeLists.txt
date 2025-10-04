cmake_minimum_required(VERSION 3.8)
project(bumperbot_planning)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(tf2_ros REQUIRED)


include_directories(include)

add_executable(dijkstra_planner src/dijkstra_planner.cpp)
ament_target_dependencies(dijkstra_planner rclcpp std_msgs nav_msgs geometry_msgs tf2_ros)

install(DIRECTORY include
  DESTINATION include
)

install(TARGETS dijkstra_planner DESTINATION lib/${PROJECT_NAME})

ament_export_include_directories(include)
ament_export_dependencies(rclcpp std_msgs nav_msgs geometry_msgs tf2_ros)
ament_package()
