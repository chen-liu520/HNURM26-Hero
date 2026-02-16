cmake_minimum_required(VERSION 3.8)
project(tf_transformer)

if (CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
    add_compile_options(-Wall -Wextra -Wpedantic)
endif ()

# find dependencies
find_package(ament_cmake_auto REQUIRED)
find_package(hnurm_interfaces REQUIRED)
ament_auto_find_build_dependencies()

# ament_auto_add_executable(sentry_manage_node
#         src/sentry_manage_node.cpp
# )

# ament_auto_add_executable(compose_node
#         src/compose_node.cpp
# )
# ament_target_dependencies(compose_node
#         hnurm_armor
#         hnurm_uart
#         hnurm_detect
#         hnurm_camera
# )


ament_auto_add_executable(tf_transformer_node
        src/tf_transformer_node.cpp
)
ament_auto_package(
        USE_SCOPED_HEADER_INSTALL_DIR
        INSTALL_TO_SHARE
        params
        launch
)
