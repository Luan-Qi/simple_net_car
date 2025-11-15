cmake_minimum_required(VERSION 3.8)
project(single_car)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)

# add_executable(car_base src/car_base.cpp)#添加可执行文件
# target_link_libraries(car_base wiringPi)
# ament_target_dependencies(car_base rclcpp)#这个是ros2为我们提供的函数，便于为ros2的东西快速链接库
# install(TARGETS car_base DESTINATION lib/${PROJECT_NAME})#把可执行文件添加到lib/${项目名称}，相当于注册节点

# test_joy.cpp
add_executable(test_joy src/test_joy.cpp)
ament_target_dependencies(test_joy rclcpp sensor_msgs)
install(TARGETS test_joy DESTINATION lib/${PROJECT_NAME})

# car_control_joy.cpp
include_directories(include/single_car/)
add_library(car_base src/car_base.cpp)
target_link_libraries(car_base wiringPi)
ament_target_dependencies(car_base rclcpp)

add_executable(car_control_joy src/car_control_joy.cpp)
target_include_directories(car_control_joy PRIVATE include/single_car/)
target_link_libraries(car_control_joy car_base)
ament_target_dependencies(car_control_joy rclcpp sensor_msgs)
install(TARGETS car_control_joy
  DESTINATION lib/${PROJECT_NAME})

# socket_server_print.cpp
add_executable(socket_server_print src/socket_server_print.cpp)
ament_target_dependencies(socket_server_print rclcpp std_msgs)
install(TARGETS socket_server_print
  DESTINATION lib/${PROJECT_NAME}
)

# camera_publisher.cpp
find_package(image_transport REQUIRED)
find_package(cv_bridge REQUIRED)
find_package(OpenCV REQUIRED)

add_executable(camera_publisher src/camera_publisher.cpp)
target_include_directories(camera_publisher PRIVATE
    ${OpenCV_INCLUDE_DIRS}
)
ament_target_dependencies(camera_publisher
    rclcpp
    image_transport
    cv_bridge
    OpenCV
    sensor_msgs
)
install(TARGETS camera_publisher
  DESTINATION lib/${PROJECT_NAME}
)

# video_server_tcp.cpp
add_executable(video_server_tcp src/video_server_tcp.cpp)
ament_target_dependencies(video_server_tcp rclcpp)
install(TARGETS video_server_tcp DESTINATION lib/${PROJECT_NAME})


add_executable(car_control_socket src/car_control_socket.cpp)
target_link_libraries(car_control_socket tinyxml2)
ament_target_dependencies(car_control_socket rclcpp sensor_msgs)
install(TARGETS car_control_socket
  DESTINATION lib/${PROJECT_NAME}
)


install(
  DIRECTORY launch
  DESTINATION share/${PROJECT_NAME}/
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
