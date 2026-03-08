cmake_minimum_required(VERSION 3.10)

# ============================================================================
# KIMICODE: 修复 CATKIN_INSTALL_INTO_PREFIX_ROOT 警告
# ============================================================================
# 问题描述: colcon 自动设置 CATKIN_INSTALL_INTO_PREFIX_ROOT 变量用于 catkin 兼容
#           但本包使用纯 ament_cmake，不识别此变量，导致 CMake 警告
# 解决方案: 如果该变量未定义但环境存在，则显式设置以避免警告
# ============================================================================
if(DEFINED ENV{COLCON_PREFIX_PATH} AND NOT DEFINED CATKIN_INSTALL_INTO_PREFIX_ROOT)
  set(CATKIN_INSTALL_INTO_PREFIX_ROOT "${CMAKE_INSTALL_PREFIX}")
endif()
# ============================================================================

project(quatro)

# 强制要求C++17
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

# ROS2 ament基础配置
find_package(ament_cmake REQUIRED)


### 编译器配置
set(CMAKE_BUILD_TYPE "Release")
set(CMAKE_CXX_FLAGS "-fexceptions -g -ggdb")
set(ignore_unused_vars "${CATKIN_INSTALL_INTO_PREFIX_ROOT}" "${CATKIN_SYMLINK_INSTALL}")
include(FindOpenMP)
if(OPENMP_FOUND)
  set(CMAKE_C_FLAGS "${CMAKE_C_FLAGS} ${OpenMP_C_FLAGS}")
  set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} ${OpenMP_CXX_FLAGS}")
  set(CMAKE_EXE_LINKER_FLAGS "${CMAKE_EXE_LINKER_FLAGS} ${OpenMP_EXE_LINKER_FLAGS}")
else()
  message(WARNING "OpenMP could not be found.")
endif()
set(CMAKE_CXX_FLAGS_RELEASE "-O3 -Wall ${CMAKE_CXX_FLAGS}")

### 查找依赖包
find_package(Eigen3 REQUIRED)
find_package(PCL REQUIRED COMPONENTS common io filters)
find_package(teaserpp REQUIRED)

### TBB配置
option(QUATRO_TBB "Enable TBB support" ON)
if(QUATRO_TBB)
  message(STATUS "Quatro with TBB")
  find_package(TBB REQUIRED)
  add_definitions(-DTBB_EN)
  include(ProcessorCount)
  ProcessorCount(N)
  if(N GREATER 4)
    add_definitions(-DTBB_PROC_NUM=${N})
  else()
    add_definitions(-DTBB_PROC_NUM=1)
  endif()
endif()

### 调试输出配置
option(QUATRO_DEBUG "Enable debug output" OFF)
if(QUATRO_DEBUG)
  add_definitions(-DQUATRO_DEBUG)
endif()

### 包含目录
include_directories(
  include
  ${Eigen3_INCLUDE_DIRS}
  ${PCL_INCLUDE_DIRS}
)

### 构建库
add_library(${PROJECT_NAME}
  src/fpfh.cc
  src/matcher.cc
  src/quatro_module.cc
)

### 链接库
target_link_libraries(${PROJECT_NAME}
  ${OpenMP_CXX_LIBRARIES}
  Eigen3::Eigen
  ${PCL_LIBRARIES}
  teaserpp::teaser_registration
  teaserpp::teaser_io
)
if(QUATRO_TBB)
  target_link_libraries(${PROJECT_NAME} TBB::tbb)
endif()

### ROS2导出配置
ament_export_include_directories(include)
ament_export_libraries(${PROJECT_NAME})
ament_export_dependencies(Eigen3 PCL teaserpp)

### 安装配置
install(
  DIRECTORY include/
  DESTINATION include
)
install(
  TARGETS ${PROJECT_NAME}
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)

### 最后调用ament
ament_package()