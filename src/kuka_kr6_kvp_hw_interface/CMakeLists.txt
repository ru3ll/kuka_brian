cmake_minimum_required(VERSION 3.8)
project(kuka_kr6_kvp_hw_interface)

#  if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
#    add_compile_options(-Wall -Wextra -Wpedantic)
#  endif()
#
#  # find dependencies
#  find_package(ament_cmake REQUIRED)
#  find_package(rclcpp REQUIRED)
# find_package(hardware_interface REQUIRED)
#  find_package(pluginlib REQUIRED)
#
#  if(BUILD_TESTING)
#    find_package(ament_lint_auto REQUIRED)
#    # the following line skips the linter which checks for copyrights
#    # comment the line when a copyright and license is added to all source files
#    set(ament_cmake_copyright_FOUND TRUE)
#    # the following line skips cpplint (only works in a git repo)
#    # comment the line when this package is in a git repo and when
#    # a copyright and license is added to all source files
#    set(ament_cmake_cpplint_FOUND TRUE)
#    ament_lint_auto_find_test_dependencies()
#  endif()
#
#  ament_package()



find_package(hardware_interface REQUIRED)
find_package(pluginlib REQUIRED)

add_library(kuka_kr6_kvp_hw_interface SHARED
  src/kuka_kr6_system.cpp
)

add_executable(kuka_kr6_node
 src/kuka_kr6_node.cpp
)
ament_target_dependencies(kuka_kr6_node 
  rclcpp 
  hardware_interface
)

target_include_directories(kuka_kr6_node PUBLIC
  include
)

target_link_libraries(kuka_kr6_node
  kuka_kr6_kvp_hw_interface
)

install(TARGETS
  kuka_kr6_node
  DESTINATION lib/${PROJECT_NAME}
)

target_include_directories(kuka_kr6_kvp_hw_interface PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)

ament_target_dependencies(kuka_kr6_kvp_hw_interface
  hardware_interface
  pluginlib
  rclcpp
)

pluginlib_export_plugin_description_file(hardware_interface kuka_kr6_kvp_hw_interface.xml)

install(
  TARGETS kuka_kr6_kvp_hw_interface
  EXPORT export_kuka_kr6_kvp_hw_interface
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)

install(DIRECTORY include/ DESTINATION include)

install(FILES kuka_kr6_kvp_hw_interface.xml
  DESTINATION share/${PROJECT_NAME}
)

ament_export_dependencies(hardware_interface pluginlib)
ament_package()

