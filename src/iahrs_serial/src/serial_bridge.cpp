/**
 * -------------------------------------------------------------------------------------------------
 * 
 * Copyright 2025 Minkyu Kil
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 *     http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 * 
 * @file    serial_bridge.cpp
 * @brief   IAHRS RB-SDA-v1 sync data publisher
 * @author  Minkyu Kil
 * @date    2025-06-17
 * @version 1.0
 *
 * -------------------------------------------------------------------------------------------------
 */

#include "iahrs_serial/serial_bridge.hpp"
#include "iahrs_serial/serial_packet.hpp"

#include "iahrs_util/exception.hpp"
#include "iahrs_util/log.hpp"

#include <functional>
#include <algorithm>
#include <chrono>
#include <cmath>

using namespace std::chrono_literals;
using std::placeholders::_1;
using std::placeholders::_2;

/**
 * @brief Default class contructor
 * @details Initializes the base Node with name "iahrs_serial_bridge".
 */
iahrs::SerialBridge::SerialBridge()
  : Node("iahrs_serial_bridge")
{
    declare_parameters();
    initialize_node();
}

/**
 * @brief Default class destructor
 * @details Destroys the Serial node, closing and deallocating the serial port.
 */
iahrs::SerialBridge::~SerialBridge()
{
    if(!serial_port_)
    {
        IAHRS_ERROR("InteractiveBridge::~InteractiveBridge() %s is deallocated already",
            port_path_.c_str());
    }
    else
    {
        serial_port_->close_port();
        serial_port_.reset();
    }
}

/**
 * @brief Pulls one IMU CSV frame, converts to SI, and publishes paired IMU & magnetic-field messages.
 * @details CSV order: ax, ay, az, gx, gy, gz, mx, my, mz, qw, qx, qy, qz.
 * Units: g→m/s², deg/s→rad/s, μT→T (1e-7). Single timestamp keeps both topics in lock-step.
 */
void iahrs::SerialBridge::timer_callback()
{
    // Receive packet
    unsigned int packet_size;
    char packet[1024];
    if(!serial_port_->receive_packet(packet, packet_size))
    {   
        return;
    }

    // Parse packet
    double data[PACKET_SIZE];
    std::sscanf(packet,
        "%lf, %lf, %lf, %lf, %lf, %lf, %lf, %lf, %lf, %lf, %lf, %lf, %lf",
        &data[ACCEL_X] , &data[ACCEL_Y] , &data[ACCEL_Z] ,
        &data[GYRO_X]  , &data[GYRO_Y]  , &data[GYRO_Z]  ,
        &data[MAGNET_X], &data[MAGNET_Y], &data[MAGNET_Z],
        &data[QUAT_W]  , &data[QUAT_X]  , &data[QUAT_Y]  , &data[QUAT_Z]
    );

    // Acceleration
    imu_data_.linear_acceleration.x = data[ACCEL_X] * GRAVITATIONAL_ACCELERATION;
    imu_data_.linear_acceleration.y = data[ACCEL_Y] * GRAVITATIONAL_ACCELERATION;
    imu_data_.linear_acceleration.z = data[ACCEL_Z] * GRAVITATIONAL_ACCELERATION;

    // Angular velocity
    imu_data_.angular_velocity.x = data[GYRO_X] * DEG2RAD;
    imu_data_.angular_velocity.y = data[GYRO_Y] * DEG2RAD;
    imu_data_.angular_velocity.z = data[GYRO_Z] * DEG2RAD;

    // Magnetic field
    magnetic_field_data_.magnetic_field.x = data[MAGNET_X] * 1e-7;
    magnetic_field_data_.magnetic_field.y = data[MAGNET_Y] * 1e-7;
    magnetic_field_data_.magnetic_field.z = data[MAGNET_Z] * 1e-7;

    // Quaternion orientation
    imu_data_.orientation.w = data[QUAT_W];
    imu_data_.orientation.x = data[QUAT_X];
    imu_data_.orientation.y = data[QUAT_Y];
    imu_data_.orientation.z = data[QUAT_Z];

    // Publish data
    imu_data_.header.stamp = this->now();
    magnetic_field_data_.header.stamp = this->now();

    imu_pub_->publish(imu_data_);
    magnetic_field_pub_->publish(magnetic_field_data_);
}

/** @brief Initializes timers, publishers, service server, and the serial port. */
void iahrs::SerialBridge::initialize_node()
{
    // Timers
    timer_ = this->create_wall_timer(10ms, std::bind(&SerialBridge::timer_callback, this));

    // Publishers
    imu_pub_ = this->create_publisher<sensor_msgs::msg::Imu>(imu_topic_,
        rclcpp::QoS(rclcpp::KeepLast(1)).best_effort().durability_volatile()
    );
    magnetic_field_pub_ = this->create_publisher<sensor_msgs::msg::MagneticField>(
        magnetic_field_topic_,
        rclcpp::QoS(rclcpp::KeepLast(1)).best_effort().durability_volatile()
    );

    // Serial port
    serial_port_ = std::make_unique<SerialPort>(port_path_, baud_rate_);
    if(!serial_port_)
    {
        throw Exception("SerialBridge::initialize_node() serial port allocation failed");
    }
    else
    {
        serial_port_->open_port();
    }

    // Sync data set
    char tmp_packet[1024];
    unsigned int tmp_packet_size;
    bool sync_settings_result = true;

    sync_settings_result = sync_settings_result && serial_port_->transmit_packet("so=1\n", strlen("so=1\n"));
    serial_port_->receive_packet(tmp_packet, tmp_packet_size);

    sync_settings_result = sync_settings_result && serial_port_->transmit_packet("sp=10\n", strlen("sp=10\n"));
    serial_port_->receive_packet(tmp_packet, tmp_packet_size);

    if(remove_gravitational_acceleration_)
    {
        // 0x00B8 = 0x0020 | 0x0008 | 0x0010 | 0x0080
        sync_settings_result = sync_settings_result && serial_port_->transmit_packet("sd=0x00B8\n", strlen("sd=0x00B8\n"));
        serial_port_->receive_packet(tmp_packet, tmp_packet_size);
    }
    else
    {
        // 0x009C = 0x0004 | 0x0008 | 0x0010 | 0x0080
        sync_settings_result = sync_settings_result && serial_port_->transmit_packet("sd=0x009C\n", strlen("sd=0x009C\n"));
        serial_port_->receive_packet(tmp_packet, tmp_packet_size);
    }

    if(!sync_settings_result)
    {
        throw Exception("SerialBridge::initialize_node() failed to set sync data");
    }

    // Initialize sensor data
    imu_data_.header.frame_id = frame_id_;
    magnetic_field_data_.header.frame_id = frame_id_;
    imu_data_.linear_acceleration_covariance = default_acceleration_covariance;
    imu_data_.angular_velocity_covariance    = default_angular_velocity_covariance;
    imu_data_.orientation_covariance         = default_orientation_covariance;
}

/** @brief Declares and retrieves ROS2 parameters for serial and IAHRS configuration. */
void iahrs::SerialBridge::declare_parameters()
{
    // Serial port
    this->declare_parameter<std::string>("port_path", "/dev/ttyUSB0");
    port_path_ = this->get_parameter("port_path").as_string();

    this->declare_parameter<int>("baud_rate", 115200);
    baud_rate_ = this->get_parameter("baud_rate").as_int();

    // Frame id and topic
    this->declare_parameter<std::string>("frame_id", "iahrs_imu");
    frame_id_ = this->get_parameter("frame_id").as_string();

    this->declare_parameter<std::string>("imu_topic", "/iahrs/imu");
    imu_topic_ = this->get_parameter("imu_topic").as_string();

    this->declare_parameter<std::string>("magnetic_field_topic", "/iahrs/magnetic_field");
    magnetic_field_topic_ = this->get_parameter("magnetic_field_topic").as_string();

    // Parameters
    this->declare_parameter<bool>("remove_gravitational_acceleration", false);
    remove_gravitational_acceleration_ = this->get_parameter(
        "remove_gravitational_acceleration").as_bool();
}