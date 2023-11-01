cmake_minimum_required(VERSION 3.8)
project(turtlesim_test)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rosidl_default_generators REQUIRED)
find_package(rclcpp_action REQUIRED)
find_package(rclcpp_components REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(turtlesim REQUIRED)
find_package(std_msgs REQUIRED)
find_package(turtlesim_interface REQUIRED)

add_executable(
        publisher
        src/pubExec.cpp
        src/speed_publisher.cpp
)
ament_target_dependencies(publisher rclcpp std_msgs geometry_msgs turtlesim)

add_executable(
        subscriber
        src/subExec.cpp
        src/pose_subscriber.cpp
)
ament_target_dependencies(subscriber rclcpp turtlesim_interface geometry_msgs turtlesim)

add_executable(
        service
        src/srvExec.cpp
        src/angle_service.cpp
)
ament_target_dependencies(service rclcpp turtlesim_interface geometry_msgs turtlesim)

install (TARGETS
        publisher
        subscriber
        service
        DESTINATION lib/${PROJECT_NAME}
)

add_library(action_server SHARED
  src/go_to_goal_action.cpp)
target_include_directories(action_server PRIVATE

<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>

<INSTALL_INTERFACE:include>)
target_compile_definitions(action_server
  PRIVATE "ACTION_GTG_CPP_BUILDING_DLL")
ament_target_dependencies(action_server
  "turtlesim_interface"
  "rclcpp"
  "rclcpp_action"
  "rclcpp_components"
  "geometry_msgs"
  "turtlesim"
  )
rclcpp_components_register_node(action_server PLUGIN "action_gtg_cpp::GTGActionServer" EXECUTABLE gtg_action_server)
install(TARGETS
  action_server
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin)


# if(BUILD_TESTING)
#   find_package(ament_lint_auto REQUIRED)
#   # the following line skips the linter which checks for copyrights
#   # comment the line when a copyright and license is added to all source files
#   set(ament_cmake_copyright_FOUND TRUE)
#   # the following line skips cpplint (only works in a git repo)
#   # comment the line when this package is in a git repo and when
#   # a copyright and license is added to all source files
#   set(ament_cmake_cpplint_FOUND TRUE)
#   ament_lint_auto_find_test_dependencies()
# endif()

ament_package()
