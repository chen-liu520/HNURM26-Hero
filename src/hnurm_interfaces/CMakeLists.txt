cmake_minimum_required(VERSION 3.8)
project(hnurm_interfaces)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake_auto REQUIRED)
ament_auto_find_build_dependencies()

file(GLOB msg_files RELATIVE ${CMAKE_CURRENT_SOURCE_DIR} msg/*/*.msg)
rosidl_generate_interfaces(
        ${PROJECT_NAME}
        ${msg_files}
        DEPENDENCIES ${${PROJECT_NAME}_FOUND_BUILD_DEPENDS}
)

ament_auto_package()
