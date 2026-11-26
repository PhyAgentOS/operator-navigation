cmake_minimum_required(VERSION 3.16)
project(navigation_sim_mppi_critics VERSION 0.1.0 LANGUAGES CXX)
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
find_package(ament_cmake REQUIRED)
find_package(nav2_mppi_controller REQUIRED)
find_package(pluginlib REQUIRED)

add_library(${PROJECT_NAME} SHARED src/segment_goal_angle_critic.cpp src/segment_goal_critic.cpp)
target_include_directories(${PROJECT_NAME} PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>)
ament_target_dependencies(${PROJECT_NAME} nav2_mppi_controller pluginlib)
pluginlib_export_plugin_description_file(nav2_mppi_controller critics.xml)
install(TARGETS ${PROJECT_NAME} EXPORT export_${PROJECT_NAME}
  ARCHIVE DESTINATION lib LIBRARY DESTINATION lib RUNTIME DESTINATION bin)
install(DIRECTORY include/ DESTINATION include)
install(FILES LICENSE README.md source-lock.json DESTINATION share/${PROJECT_NAME})
install(FILES env/local_setup.bash DESTINATION .)

if(BUILD_TESTING)
  enable_testing()
  add_executable(test_segment_goal_angle tests/test_segment_goal_angle.cpp)
  target_link_libraries(test_segment_goal_angle ${PROJECT_NAME})
  ament_target_dependencies(test_segment_goal_angle nav2_mppi_controller pluginlib)
  add_test(NAME segment_goal_angle_regression COMMAND test_segment_goal_angle)
  # Deliberately do not link our plugin library: prove runtime discovery.
  add_executable(test_plugin_loading tests/test_plugin_loading.cpp)
  ament_target_dependencies(test_plugin_loading nav2_mppi_controller pluginlib)
  add_test(NAME installed_plugin_discovery COMMAND test_plugin_loading)
endif()
ament_export_targets(export_${PROJECT_NAME} HAS_LIBRARY_TARGET)
ament_export_dependencies(nav2_mppi_controller pluginlib)
ament_package()
