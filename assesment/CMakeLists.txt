cmake_minimum_required(VERSION 3.8)
project(assesment)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(PkgConfig REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_action REQUIRED)
find_package(nav2_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(action_msgs REQUIRED)
find_package(rosidl_default_generators REQUIRED)

find_package(wormhole_action REQUIRED)

pkg_check_modules(LIBPQ REQUIRED libpq)

add_library(database_manager src/database_manager.cpp)
target_include_directories(database_manager PUBLIC
  include
  ${LIBPQ_INCLUDE_DIRS}
)

# add_executable(database_manager src/database_manager.cpp)
target_link_libraries(database_manager ${LIBPQ_LIBRARIES})
ament_target_dependencies(database_manager
  rclcpp
  LIBPQ
)

# Build the action server executable
add_executable(multi_map_action_server src/multi_map_action_server.cpp)

ament_target_dependencies(multi_map_action_server
  rclcpp
  rclcpp_action
  geometry_msgs
  nav_msgs
  nav2_msgs
  action_msgs
  wormhole_action
)

# Link libraries
target_link_libraries(multi_map_action_server
  database_manager
  ${LIBPQ_LIBRARIES}
)

# Install the node executable
install(TARGETS 
  multi_map_action_server
  DESTINATION lib/${PROJECT_NAME}
)

# Install targets
install(TARGETS 
  multi_map_action_server
  database_manager
  DESTINATION lib/${PROJECT_NAME}
)

install(
  DIRECTORY config include launch maps src worlds
  DESTINATION share/${PROJECT_NAME}
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  
  set(ament_cmake_copyright_FOUND TRUE)
  
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

ament_export_include_directories(include)
ament_export_libraries(database_manager)

ament_package()