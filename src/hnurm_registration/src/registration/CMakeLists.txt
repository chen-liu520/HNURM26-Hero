cmake_minimum_required(VERSION 3.8)

# ============================================================================
# KIMICODE: 修复 CMP0074 策略警告
# ============================================================================
# 问题描述: CMake 检测到 PCL_ROOT 环境变量，但策略 CMP0074 未设置
#           导致 find_package 忽略 <PackageName>_ROOT 变量，产生开发者警告
# 解决方案: 显式设置 CMP0074 为 NEW，使 find_package 正确使用 PCL_ROOT
# 参考资料: cmake --help-policy CMP0074
# ============================================================================
if(POLICY CMP0074)
  cmake_policy(SET CMP0074 NEW)
endif()
# ============================================================================

project(registration)

set(CMAKE_CXX_STANDARD 17)   # 设置 C++ 标准为 C++17，且强制要求支持
set(CMAKE_CXX_STANDARD_REQUIRED ON)
add_definitions(-Wall -Werror) # 启用所有警告，并将警告视为错误

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake_auto REQUIRED)
ament_auto_find_build_dependencies(REQUIRED
  rclcpp 
  sensor_msgs 
  tf2 
  tf2_ros 
  tf2_eigen
  pcl_conversions
  geometry_msgs
  pcl_ros
  quatro
)
find_package(PCL REQUIRED)
find_package(OpenMP REQUIRED)
find_package(Eigen3 REQUIRED)
find_package(teaserpp REQUIRED)
find_package(small_gicp REQUIRED)

if(OpenMP_CXX_FOUND)
  add_definitions(-DOPENMP_AVAILABLE)  # 定义宏告诉small_gicp已启用OpenMP
  set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} ${OpenMP_CXX_FLAGS}") # 添加编译标志
endif()

ament_auto_add_executable(registration_node 
                src/main.cpp 
                src/registration_node.cpp) # 创建可执行文件

target_include_directories(registration_node PRIVATE
  ${PCL_INCLUDE_DIRS}
)

target_link_libraries(registration_node
  ${PCL_LIBRARIES}
  Eigen3::Eigen
  OpenMP::OpenMP_CXX
  teaserpp::teaser_registration 
  teaserpp::teaser_io
  small_gicp::small_gicp
  #pcl_ros::pcl_ros
)

# ============================================================================
# KIMICODE: 修复头文件安装路径建议警告
# ============================================================================
# 问题描述: ament_auto_package 默认将头文件安装到 include/
#           但 ROS 2 Kilted Kaiju+ 推荐安装到 include/registration/ 以支持 scoped include
#           即: #include <registration/header.hpp> 而非 #include <header.hpp>
# 解决方案: 使用 USE_SCOPED_HEADER_INSTALL_DIR 选项启用 scoped 安装路径
#           在 Kilted 之前版本，此选项显式启用新行为；Kilted+ 将成为默认
# ============================================================================
ament_auto_package(
        USE_SCOPED_HEADER_INSTALL_DIR  # 【KIMICODE】: 启用 scoped 头文件安装路径
        INSTALL_TO_SHARE
        launch   # 安装 launch 文件夹
        params   # 安装 params 文件夹
        rviz     # 安装 rviz 配置文件夹
)
# ============================================================================


