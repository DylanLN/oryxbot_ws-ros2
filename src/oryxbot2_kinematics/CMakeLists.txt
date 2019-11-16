cmake_minimum_required(VERSION 3.5)
project(oryxbot2_kinematics)

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
find_package(rclcpp_components REQUIRED)
find_package(std_msgs REQUIRED)
find_package(oryxbot2_msgs REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(tf2 REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(orocos_kdl REQUIRED)

find_package(Boost REQUIRED COMPONENTS system thread)

include_directories(include)
# uncomment the following section in order to fill in
# further dependencies manually.
# find_package(<dependency> REQUIRED)

## Declare a C++ library
# add_library(${PROJECT_NAME}
#   src/oryxbot_kinematics.cpp
 #  ${Boost_LIRARIES}
# )
 add_library(${PROJECT_NAME} SHARED
   src/oryxbot2_kinematics.cpp
   ${Boost_LIRARIES}
 )
target_compile_definitions(${PROJECT_NAME}
  PRIVATE "MINIMAL_COMPOSITION_DLL")
ament_target_dependencies(${PROJECT_NAME} rclcpp rclcpp_components std_msgs geometry_msgs oryxbot2_msgs Boost)

# This package installs libraries without exporting them.
# Export the library path to ensure that the installed libraries are available.
if(NOT WIN32)
  ament_environment_hooks(
    "${ament_cmake_package_templates_ENVIRONMENT_HOOK_LIBRARY_PATH}"
    )
endif()


add_executable(oryxbot2_kinematic_node src/oryxbot2_kinematics_node.cpp)
target_link_libraries(oryxbot2_kinematic_node ${catkin_LIBRARIES} oryxbot2_kinematics)
ament_target_dependencies(oryxbot2_kinematic_node Boost ${PROJECT_NAME} rclcpp std_msgs geometry_msgs oryxbot2_msgs)

target_include_directories(oryxbot2_kinematic_node PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>)

  add_executable(odometry2_node src/odometry2.cpp)
  target_link_libraries(odometry2_node ${catkin_LIBRARIES})
  ament_target_dependencies(odometry2_node ${PROJECT_NAME} rclcpp nav_msgs std_msgs geometry_msgs oryxbot2_msgs tf2_ros tf2)

  target_include_directories(odometry2_node PUBLIC
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
    $<INSTALL_INTERFACE:include>)

install(TARGETS
  ${PROJECT_NAME}
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin)

  install(TARGETS 
    odometry2_node
    oryxbot2_kinematic_node
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
