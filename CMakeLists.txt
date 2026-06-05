cmake_minimum_required(VERSION 3.8)
project(sura_cameras)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(camera_info_manager REQUIRED)
find_package(cv_bridge REQUIRED)
find_package(OpenCV REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)

add_executable(image_decimator src/image_decimator.cpp)
ament_target_dependencies(image_decimator
  cv_bridge
  rclcpp
  sensor_msgs
)
target_link_libraries(image_decimator ${OpenCV_LIBRARIES})

add_executable(sim_camera_republisher src/sim_camera_republisher.cpp)
ament_target_dependencies(sim_camera_republisher
  camera_info_manager
  rclcpp
  sensor_msgs
)

install(TARGETS
  image_decimator
  sim_camera_republisher
  DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY
  config
  launch
  DESTINATION share/${PROJECT_NAME}
)

ament_package()
