cmake_minimum_required(VERSION 3.8)
project(mobile_robot_control)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(rclcpp REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)

add_library(angle_utils
  src/angle_utils.cpp
)

target_compile_features(angle_utils PUBLIC cxx_std_17)

target_include_directories(angle_utils
  PUBLIC
    "$<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>"
    "$<INSTALL_INTERFACE:include>"
)

add_library(control_law
  src/control_law.cpp
)

target_compile_features(control_law PUBLIC cxx_std_17)

target_include_directories(control_law
  PUBLIC
    "$<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>"
    "$<INSTALL_INTERFACE:include>"
)

target_link_libraries(control_law
  PUBLIC angle_utils
)

add_executable(path_tracking_controller
  src/path_tracking_controller_node.cpp
)

target_compile_features(path_tracking_controller PUBLIC cxx_std_17)

target_include_directories(path_tracking_controller
  PRIVATE
    "${CMAKE_CURRENT_SOURCE_DIR}/include"
)

target_link_libraries(path_tracking_controller
  control_law
)

ament_target_dependencies(path_tracking_controller
  geometry_msgs
  nav_msgs
  rclcpp
  tf2
  tf2_geometry_msgs
)

install(
  TARGETS angle_utils control_law
  EXPORT export_${PROJECT_NAME}
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
  INCLUDES DESTINATION include
)

install(
  TARGETS path_tracking_controller
  DESTINATION lib/${PROJECT_NAME}
)

install(
  DIRECTORY include/
  DESTINATION include
)

install(
  DIRECTORY config launch
  DESTINATION share/${PROJECT_NAME}
)

if(BUILD_TESTING)
  find_package(ament_cmake_gtest REQUIRED)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()

  ament_add_gtest(test_angle_utils
    test/test_angle_utils.cpp
  )
  target_link_libraries(test_angle_utils angle_utils)

  ament_add_gtest(test_path_types
    test/test_path_types.cpp
  )
  target_include_directories(test_path_types
    PRIVATE
      "${CMAKE_CURRENT_SOURCE_DIR}/include"
  )
  target_compile_features(test_path_types PUBLIC cxx_std_17)

  ament_add_gtest(test_waypoint_selector
    test/test_waypoint_selector.cpp
  )
  target_include_directories(test_waypoint_selector
    PRIVATE
      "${CMAKE_CURRENT_SOURCE_DIR}/include"
  )
  target_compile_features(test_waypoint_selector PUBLIC cxx_std_17)

  ament_add_gtest(test_control_law
    test/test_control_law.cpp
  )
  target_link_libraries(test_control_law control_law)
endif()

ament_export_targets(export_${PROJECT_NAME} HAS_LIBRARY_TARGET)
ament_export_include_directories(include)
ament_export_libraries(angle_utils control_law)

ament_package()
