cmake_minimum_required(VERSION 3.14)
project(hnurm_uart)

set(CMAKE_CXX_STANDARD 17)

find_package(ament_cmake_auto REQUIRED)
ament_auto_find_build_dependencies()

ament_auto_add_library(hnurm_crc SHARED DIRECTORY src/CRC/)
ament_auto_add_library(hnurm_protocol SHARED DIRECTORY src/Protocol/)
ament_auto_add_library(hnurm_serial SHARED DIRECTORY src/Serial/)

ament_auto_add_library(hnurm_uart src/uart_node.cpp)
target_link_libraries(hnurm_uart hnurm_crc hnurm_protocol hnurm_serial)

ament_auto_add_executable(hnurm_uart_node src/uart_node.cpp src/main.cpp)
target_link_libraries(hnurm_uart_node hnurm_uart)

ament_auto_package(
        INSTALL_TO_SHARE
        launch
        params
)
