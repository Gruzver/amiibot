cmake_minimum_required(VERSION 3.10)
project(face_detection)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(OpenCV REQUIRED)  # Asegúrate de que OpenCV esté instalado
find_package(cv_bridge REQUIRED)  # Si quieres integrar OpenCV con ROS
find_package(serial REQUIRED)
# uncomment the following section in order to fill in
# further dependencies manually.
# find_package(<dependency> REQUIRED)

# Create the executable
add_executable(face_publisher src/face_publisher.cpp)

# Link dependencies
ament_target_dependencies(
  face_publisher
  rclcpp
  std_msgs
  OpenCV
  cv_bridge
)

add_executable(face_subscriber src/face_subscriber.cpp)
ament_target_dependencies(
  face_subscriber
  rclcpp
  std_msgs
  serial
)


# Install the executable
install(TARGETS
  face_publisher
  face_subscriber
  DESTINATION lib/${PROJECT_NAME}
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
