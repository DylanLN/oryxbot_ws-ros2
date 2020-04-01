cmake_minimum_required(VERSION 3.5)
project(tutorials_tf)

# Default to C99
if(NOT CMAKE_C_STANDARD)
  set(CMAKE_C_STANDARD 99)
endif()

# Default to C++14
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 14)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(std_msgs REQUIRED)
find_package(rclcpp REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(turtlesim REQUIRED)
find_package(geometry_msgs REQUIRED)


add_executable(turtle_tf_broadcaster src/turtle_tf_broadcaster.cpp)
ament_target_dependencies(turtle_tf_broadcaster "rclcpp" "tf2" "tf2_ros" "turtlesim")
add_executable(turtle_tf_listener src/turtle_tf_listener.cpp)
ament_target_dependencies(turtle_tf_listener "rclcpp" "tf2" "tf2_ros" "turtlesim" "geometry_msgs")

target_link_libraries(
  turtle_tf_broadcaster
  ${catkin_LIBRARIES})

  install(TARGETS
    turtle_tf_broadcaster
    turtle_tf_listener
    EXPORT export_${PROJECT_NAME}
    DESTINATION lib/${PROJECT_NAME})


if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  # the following line skips the linter which checks for copyrights
  # uncomment the line when a copyright and license is not present in all source files
  #set(ament_cmake_copyright_FOUND TRUE)
  # the following line skips cpplint (only works in a git repo)
  # uncomment the line when this package is not in a git repo
  #set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
