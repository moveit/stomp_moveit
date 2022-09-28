cmake_minimum_required(VERSION 3.8)
project(stomp_moveit)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic -Wno-unused-parameter)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(stomp REQUIRED)
find_package(moveit_core REQUIRED)
find_package(moveit_ros REQUIRED)
find_package(moveit_visual_tools REQUIRED)
find_package(generate_parameter_library REQUIRED)

generate_parameter_library(stomp_moveit_parameters res/stomp_moveit.yaml)

include_directories(stomp_moveit_example PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>)

# Planner Plugin
add_library(stomp_moveit_plugin SHARED
  src/stomp_moveit_planner_plugin.cpp
  src/stomp_moveit_planning_context.cpp
)
ament_target_dependencies(stomp_moveit_plugin
  moveit_core
)
target_link_libraries(stomp_moveit_plugin stomp::stomp stomp_moveit_parameters)

pluginlib_export_plugin_description_file(moveit_core stomp_moveit_plugin_description.xml)

install(TARGETS stomp_moveit_plugin stomp_moveit_parameters
  EXPORT ${PROJECT_NAME}Targets
  LIBRARY DESTINATION lib
  ARCHIVE DESTINATION lib
  RUNTIME DESTINATION bin
  INCLUDES DESTINATION include
)

# Demo Executable
add_executable(stomp_moveit_example src/stomp_moveit_example.cpp)
ament_target_dependencies(stomp_moveit_example
  moveit_core
  moveit_ros
  moveit_visual_tools
)
target_link_libraries(stomp_moveit_example stomp::stomp)

install(TARGETS stomp_moveit_example
  DESTINATION lib/${PROJECT_NAME})

install(DIRECTORY launch DESTINATION share/${PROJECT_NAME})

ament_export_targets(${PROJECT_NAME}Targets HAS_LIBRARY_TARGET)
ament_export_dependencies(moveit_core stomp generate_parameter_library)

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
