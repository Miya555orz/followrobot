# bringup_pkg CMakeLists.txt
# 纯启动包 — 无编译内容，仅安装 launch/config/rviz 等运行资源

# Ubuntu 22.04 / ROS 2 Humble ships CMake 3.22. file(CHMOD), used below to
# make Windows-originated scripts executable under symlink-install, requires
# CMake 3.19 or newer.
cmake_minimum_required(VERSION 3.19)
project(bringup_pkg)

# ament_cmake：ROS2 C++ 构建工具（纯资源包也需要声明）
find_package(ament_cmake REQUIRED)

# 将 launch、config、rviz 目录安装到包共享目录
# 运行时通过 FindPackageShare 引用这些资源
install(DIRECTORY launch config rviz
  DESTINATION share/${PROJECT_NAME}
)

# Jetson real-hardware convenience entrypoints. Copy through the build tree
# and force executable permissions so `colcon --symlink-install` does not
# inherit a non-executable source bit from a Windows checkout.
set(FCR_START_SCRIPTS
  start_fcr.sh
  start_fcr_gimbal.sh
)
file(MAKE_DIRECTORY "${CMAKE_CURRENT_BINARY_DIR}/scripts")
foreach(script_name IN LISTS FCR_START_SCRIPTS)
  configure_file(
    "${CMAKE_CURRENT_SOURCE_DIR}/scripts/${script_name}"
    "${CMAKE_CURRENT_BINARY_DIR}/scripts/${script_name}"
    @ONLY
    NEWLINE_STYLE UNIX
  )
  file(CHMOD "${CMAKE_CURRENT_BINARY_DIR}/scripts/${script_name}"
    PERMISSIONS
      OWNER_READ OWNER_WRITE OWNER_EXECUTE
      GROUP_READ GROUP_EXECUTE
      WORLD_READ WORLD_EXECUTE
  )
endforeach()

install(PROGRAMS
  ${CMAKE_CURRENT_BINARY_DIR}/scripts/start_fcr.sh
  ${CMAKE_CURRENT_BINARY_DIR}/scripts/start_fcr_gimbal.sh
  DESTINATION lib/${PROJECT_NAME}
)

ament_package()
