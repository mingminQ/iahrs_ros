# --------------------------------------------------------------------------------------------------
# SPDX-License-Identifier: Apache-2.0
# Copyright (c) 2025 Minkyu Kil
# --------------------------------------------------------------------------------------------------
cmake_minimum_required(VERSION 3.8)
project(iahrs_gui)

# --------------------------------------------------------------------------------------------------
# Ensure C++17 standard is used if not already set
# --------------------------------------------------------------------------------------------------
if(NOT CMAKE_CXX_STANDARD)
    set(CMAKE_CXX_STANDARD 17)
endif()

# --------------------------------------------------------------------------------------------------
# Add extra warning flags for GCC and Clang
# --------------------------------------------------------------------------------------------------
if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
    add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# --------------------------------------------------------------------------------------------------
# Enable Qt auto-processing and set UIC
# --------------------------------------------------------------------------------------------------
set(CMAKE_AUTOMOC ON)
set(CMAKE_AUTOUIC ON)
set(CMAKE_INCLUDE_CURRENT_DIR ON)
set(CMAKE_AUTOUIC_SEARCH_PATHS ${CMAKE_CURRENT_SOURCE_DIR}/ui)

# --------------------------------------------------------------------------------------------------
# Find required ROS2 and message packages
# --------------------------------------------------------------------------------------------------
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(iahrs_util REQUIRED)
find_package(iahrs_msgs REQUIRED)
find_package(tf2 REQUIRED)
find_package(Qt5Core REQUIRED)
find_package(Qt5Widgets REQUIRED)

# --------------------------------------------------------------------------------------------------
# Include local header files
# --------------------------------------------------------------------------------------------------
include_directories(
    include
)

# --------------------------------------------------------------------------------------------------
# IAHRS sensor monitor
# --------------------------------------------------------------------------------------------------
set(IAHRS_SENSOR_MONITOR_SOURCE
    include/iahrs_gui/sensor_monitor.hpp
    src/sensor_monitor.cpp
    ui/sensor_monitor.ui
)

add_executable(sensor_monitor
    node/sensor_monitor_node.cpp
    ${IAHRS_SENSOR_MONITOR_SOURCE}
)

ament_target_dependencies(sensor_monitor
    rclcpp
    sensor_msgs
    iahrs_util
    iahrs_msgs
    tf2
)

target_include_directories(sensor_monitor
    PUBLIC
        $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
        $<INSTALL_INTERFACE:include>
)

target_link_libraries(sensor_monitor
    Qt5::Core
    Qt5::Widgets
)

install(
    TARGETS sensor_monitor
    DESTINATION lib/${PROJECT_NAME}
)

# --------------------------------------------------------------------------------------------------
# Register the package with ament
# --------------------------------------------------------------------------------------------------
ament_export_include_directories(include)
ament_export_libraries(${PROJECT_NAME})
ament_package()