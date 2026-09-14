cmake_minimum_required(VERSION 3.8)
project(udp_camera_ros)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_lifecycle REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(cv_bridge REQUIRED)
find_package(image_transport REQUIRED)
find_package(compressed_image_transport REQUIRED)
find_package(ament_index_cpp REQUIRED)
find_package(yaml_cpp_vendor REQUIRED)
find_package(OpenCV REQUIRED)

add_executable(camera_node
  src/camera_node.cpp
  src/camera.cpp
  src/udp_stream.cpp
  src/camera_info.cpp
)

target_include_directories(camera_node PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)
target_include_directories(camera_node SYSTEM PUBLIC ${OpenCV_INCLUDE_DIRS})
target_compile_features(camera_node PUBLIC cxx_std_17)

ament_target_dependencies(camera_node
  rclcpp
  rclcpp_lifecycle
  sensor_msgs
  cv_bridge
  image_transport
  compressed_image_transport
  ament_index_cpp
  yaml_cpp_vendor
)
target_link_libraries(camera_node ${OpenCV_LIBRARIES} yaml-cpp)

install(TARGETS camera_node
  DESTINATION lib/${PROJECT_NAME})

install(DIRECTORY include/
  DESTINATION include)

install(DIRECTORY launch config
  DESTINATION share/${PROJECT_NAME})

ament_package()
