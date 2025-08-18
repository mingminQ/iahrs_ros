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
 * @file    interactive_bridge.cpp
 * @brief   IAHRS RB-SDA-v1 interactive interface
 * @author  Minkyu Kil
 * @date    2025-06-17
 * @version 1.0
 *
 * -------------------------------------------------------------------------------------------------
 */

#include "iahrs_serial/interactive_bridge.hpp"
#include "iahrs_util/exception.hpp"
#include "iahrs_util/timer.hpp"
#include "iahrs_util/log.hpp"

/**
 * @brief Constructor for the interactive interface node
 * @details Initializes node name, declares parameters, and opens the serial port.
 */
iahrs::InteractiveBridge::InteractiveBridge()
  : Node("iahrs_interactive")
{
    declare_parameters();
    initialize_node();
}

/**
 * @brief Destructor
 * @details Closes and deallocates the serial port when node is destroyed.
 */
iahrs::InteractiveBridge::~InteractiveBridge()
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
 * @brief Send a command and log the raw response
 * @param[in] command Null-terminated command string to transmit (e.g., "g\n")
 * @details Transmits the given command and logs the received packet size and contents.
 */
void iahrs::InteractiveBridge::query(const char *command)
{
    char output[1024];
    unsigned int packet_size;

    // Timer start
    rx_timer_.start();

    if(!serial_port_->transmit_packet(command, strlen(command)))
    {
        return;
    }
    if(serial_port_->receive_packet(output, packet_size))
    {
        IAHRS_INFO("InteractiveBridge::query() packet size: %u, %s", 
            packet_size, output);
    }
    
    // Timer end
    rx_timer_.end();
}

/**
 * @brief Initialize the serial port connection
 * @details Creates and opens the SerialPort instance using configured parameters.
 */
void iahrs::InteractiveBridge::initialize_node()
{
    // Serial port
    serial_port_ = std::make_unique<SerialPort>(port_path_, baud_rate_);
    if(!serial_port_)
    {
        throw Exception("InteractiveBridge::initialize_node() serial port allocation failed");
    }
    else
    {
        serial_port_->open_port();
    }
}

/**
 * @brief Declare ROS2 parameters for serial configuration
 * @details Declares and reads 'port_path' and 'baud_rate' parameters from the parameter server.
 */
void iahrs::InteractiveBridge::declare_parameters()
{
    // Serial port
    this->declare_parameter<std::string>("port_path", "/dev/ttyUSB0");
    port_path_ = this->get_parameter("port_path").as_string();

    this->declare_parameter<int>("baud_rate", 115200);
    baud_rate_ = this->get_parameter("baud_rate").as_int();
}