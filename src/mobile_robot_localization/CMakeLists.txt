cmake_minimum_required(VERSION 3.8)
project(mobile_robot_localization)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
# uncomment the following section in order to fill in
# further dependencies manually.
# find_package(<dependency> REQUIRED)

add_executable(state_estimator
  src/state_estimator_node.cpp
)

target_compile_features(state_estimator PUBLIC cxx_std_17)

ament_target_dependencies(state_estimator
  nav_msgs
  rclcpp
  sensor_msgs
)

install(
  TARGETS state_estimator
  DESTINATION lib/${PROJECT_NAME}
)

install(
  DIRECTORY include/
  DESTINATION include
)

ament_export_include_directories(include)

if(BUILD_TESTING)
  find_package(ament_cmake_gtest REQUIRED)
  find_package(ament_lint_auto REQUIRED)

  ament_add_gtest(test_planar_state
    test/test_planar_state.cpp
  )
  target_include_directories(test_planar_state PRIVATE
    include
  )

  ament_add_gtest(test_angle_utils
    test/test_angle_utils.cpp
  )
  target_include_directories(test_angle_utils PRIVATE
    include
  )

  ament_add_gtest(test_state_predictor
    test/test_state_predictor.cpp
  )
  target_include_directories(test_state_predictor PRIVATE
    include
  )

  ament_add_gtest(test_measurement_fusion
    test/test_measurement_fusion.cpp
  )
  target_include_directories(test_measurement_fusion PRIVATE
    include
  )

  ament_add_gtest(test_planar_estimator
    test/test_planar_estimator.cpp
  )
  target_include_directories(test_planar_estimator PRIVATE
    include
  )

  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
