cmake_minimum_required(VERSION 3.8)
project(ros2_worm_multiplayer)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-g -Wall -Wextra -Wpedantic -Wno-sign-compare)
endif()

include_directories(include)

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(Curses REQUIRED)
find_package(std_msgs REQUIRED) # only needed for testing purposes - mario
find_package(rosidl_default_generators REQUIRED) # testing - Joschie
find_package(ament_cmake_python REQUIRED)
find_package(rclpy REQUIRED)
#find_package(ros2_worm_multiplayer REQUIRED)

set(msg_files
  "msg/PlayerInput.msg"
  "msg/Board.msg"
  "msg/Row.msg"
  "msg/Element.msg"
  "msg/Direction.msg"
)

set(srv_files
  "srv/JoinServer.srv"
)

# Messages - Peter
rosidl_generate_interfaces(${PROJECT_NAME}
	${msg_files}
  ${srv_files}
)

# Display Node
add_executable(worm_display_node
	src/worm_display_node.cpp
)
ament_target_dependencies(worm_display_node rclcpp std_msgs)

# Navigation Node
add_executable(worm_navigation_node
  src/worm_navigation_node.cpp
  src/prep.c
)
ament_target_dependencies(worm_navigation_node rclcpp std_msgs)

# Grid Node
add_executable(worm_grid_node
  src/worm_grid_node.cpp
)
ament_target_dependencies(worm_grid_node rclcpp std_msgs)

#######################################
target_include_directories(worm_display_node
  PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)

# 
install(TARGETS 
  worm_display_node
  worm_navigation_node
  worm_grid_node
  DESTINATION lib/${PROJECT_NAME}
)

# type support for custom msgs
rosidl_get_typesupport_target(cpp_typesupport_target
  ${PROJECT_NAME} rosidl_typesupport_cpp)

# add your nodes here that require custom msg types
target_link_libraries(worm_navigation_node "${cpp_typesupport_target}")
target_link_libraries(worm_grid_node "${cpp_typesupport_target}")
target_link_libraries(worm_display_node "${cpp_typesupport_target}")

# add your nodes here that require ncurses
target_link_libraries(worm_display_node ncurses)
target_link_libraries(worm_navigation_node ncurses)



ament_python_install_package(${PROJECT_NAME}_python)
install(PROGRAMS
  scripts/worm_display_node.py
  DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY
  launch
  DESTINATION share/${PROJECT_NAME}/
)

ament_package()
