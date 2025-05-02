cmake_minimum_required(VERSION 3.5)
project(jet_controller)

# Default to C99
if(NOT CMAKE_C_STANDARD)
  set(CMAKE_C_STANDARD 99)
endif()

# Default to C++14
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(mujoco_ros_msgs REQUIRED)
find_package(Eigen3 REQUIRED)
find_package(tf2 REQUIRED)
find_package(jet_lib REQUIRED)
find_package(jet_description REQUIRED)
find_package(jet_msgs REQUIRED)

find_package(jet_cc QUIET)
if(jet_cc_FOUND)
  message(" -- jet_cc found")
  add_definitions(-DCOMPILE_JETCC)
endif()

include_directories(
  include
  ${rclcpp_INCLUDE_DIRS}
  ${eigen_conversions_INCLUDE_DIRS}
  ${sensor_msgs_INCLUDE_DIRS}
  ${std_msgs_INCLUDE_DIRS}
  ${mujoco_ros_msgs_INCLUDE_DIRS}
  ${tf2_INCLUDE_DIRS}
  # ${tf2_ros_INCLUDE_DIRS}
  ${jet_lib_INCLUDE_DIRS}
  ${jet_description_INCLUDE_DIRS}
  ${jet_msgs_INCLUDE_DIRS}
  ${jet_cc_INCLUDE_DIRS}
)

add_library(${PROJECT_NAME}_lib SHARED
  src/state_manager.cpp
  src/jet_controller.cpp
)

ament_target_dependencies(${PROJECT_NAME}_lib
  rclcpp
  eigen_conversions
  sensor_msgs
  std_msgs
  mujoco_ros_msgs
  tf2
  # tf2_ros
  jet_lib
  jet_description
  jet_msgs
)

target_link_libraries(${PROJECT_NAME}_lib
  ${jet_cc_LIBRARIES}
  pthread
)

add_executable(${PROJECT_NAME} src/jet_controller.cpp)
add_executable(shm_reset src/shm_reset.cpp)

ament_target_dependencies(${PROJECT_NAME} rclcpp)
ament_target_dependencies(shm_reset rclcpp)

target_link_libraries(${PROJECT_NAME}
  ${PROJECT_NAME}_lib
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()
endif()

# # Install targets and headers
# install(
#   DIRECTORY include/
#   DESTINATION include
# )
# install(
#   TARGETS ${PROJECT_NAME}_lib
#   EXPORT export_${PROJECT_NAME}_lib
#   ARCHIVE DESTINATION lib
#   LIBRARY DESTINATION lib
#   RUNTIME DESTINATION bin
# )
# install(
#   TARGETS ${PROJECT_NAME} shm_reset
#   RUNTIME DESTINATION lib/${PROJECT_NAME}
# )
# install(
#   EXPORT export_${PROJECT_NAME}_lib
#   NAMESPACE ${PROJECT_NAME}_lib::
#   DESTINATION share/${PROJECT_NAME}/cmake
# )

# # Export for ament
# ament_export_targets(export_${PROJECT_NAME}_lib HAS_LIBRARY_TARGET)
# ament_export_dependencies(
#   rclcpp
#   eigen_conversions
#   sensor_msgs
#   std_msgs
#   mujoco_ros_msgs
#   tf2
#   # tf2_ros
#   jet_lib
#   jet_description
#   jet_msgs
#   $<$<BOOL:${jet_cc_FOUND}>:jet_cc>
# )

# ament_export_include_directories(include)
# ament_export_libraries(${PROJECT_NAME}_lib)
ament_package()
