cmake_minimum_required(VERSION 3.8)
project(trigger_registration)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_srvs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(std_msgs REQUIRED)

add_executable(trigger_hero_node src/trigger_hero_node.cpp)
target_include_directories(trigger_hero_node PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>)
target_compile_features(trigger_hero_node PUBLIC c_std_99 cxx_std_17)
ament_target_dependencies(
  trigger_hero_node
  "rclcpp"
  "std_srvs"
  "std_msgs"
)

install(TARGETS trigger_hero_node
  DESTINATION lib/${PROJECT_NAME})

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()