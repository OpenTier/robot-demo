cmake_minimum_required(VERSION 3.8)
project(cloud_comm_node)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies ROS 2
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
# Zenoh packages
find_package(zenohc REQUIRED)
find_package(zenohcxx REQUIRED)
# Protobuf
find_package(Protobuf REQUIRED)

# deps
find_package(edge_device_msgs REQUIRED)

# Generate the protobuf files
SET(PROTOBUF_DIR ${CMAKE_CURRENT_SOURCE_DIR}/../../../vehicle-cloud-api/proto)
protobuf_generate_cpp(PROTO_SRCS PROTO_HDRS ${PROTOBUF_DIR}/vehicle_cloud_events.proto ${PROTOBUF_DIR}/robot_soccer.proto)

# add executable
add_executable(${PROJECT_NAME} src/main.cpp ${PROTO_SRCS})
ament_target_dependencies(${PROJECT_NAME} rclcpp std_msgs edge_device_msgs)

# Link libraries
target_link_libraries(${PROJECT_NAME} zenohcxx::zenohc ${Protobuf_LIBRARIES})
# Add includes
target_include_directories(${PROJECT_NAME} PUBLIC ${CMAKE_CURRENT_BINARY_DIR})
target_include_directories(${PROJECT_NAME} PUBLIC
  ${CMAKE_CURRENT_SOURCE_DIR}/include
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  # the following line skips the linter which checks for copyrights
  # comment the line when a copyright and license is added to all source files
  set(ament_cmake_copyright_FOUND TRUE)
  # the following line skips cpplint (only works in a git repo)
  # comment the line when this package is in a git repo and when
  # a copyright and license is added to all source files
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()

install(TARGETS
  ${PROJECT_NAME}
  DESTINATION lib/${PROJECT_NAME}
)
