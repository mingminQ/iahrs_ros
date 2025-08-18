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
 * @file    interactive_bridge.hpp
 * @brief   IAHRS RB-SDA-v1 interactive interface
 * @author  Minkyu Kil
 * @date    2025-06-17
 * @version 1.0
 *
 * -------------------------------------------------------------------------------------------------
 */

#ifndef IAHRS_SERIAL_INTERACTIVE_BRIDGE_HPP_
#define IAHRS_SERIAL_INTERACTIVE_BRIDGE_HPP_

#include "iahrs_serial/serial_port.hpp"
#include "iahrs_util/timer.hpp"

#include "rclcpp/rclcpp.hpp"

#include <string>
#include <memory>

namespace iahrs
{
    /** 
     * @brief 
     */
    class InteractiveBridge : public rclcpp::Node
    {
    // "InteractiveBridge" member functions
    public:

        /**
         * @brief Constructor for the Interactive interface node
         * @details Initializes node name, declares parameters, and opens the serial port.
         */
        InteractiveBridge();

        /**
         * @brief Destructor
         * @details Closes and deallocates the serial port when node is destroyed.
         */
        ~InteractiveBridge();

        /**
         * @brief Send a command and log the raw response
         * @param[in] command Null-terminated command string to transmit (e.g., "g\n")
         * @details Transmits the given command and logs the received packet size and contents.
         */
        void query(const char *command);

    private:

        /**
         * @brief Initialize the serial port connection
         * @details Creates and opens the SerialPort instance using configured parameters.
         */
        void initialize_node();

        /**
         * @brief Declare ROS2 parameters for serial configuration
         * @details Declares and reads 'port_path' and 'baud_rate' parameters from the parameter server.
         */
        void declare_parameters();

    // "InteractiveBridge" member variables
    private:
    
        // Timer
        Timer rx_timer_;

        // Serial port
        std::unique_ptr<SerialPort> serial_port_;

        // Serial port parameters
        std::string port_path_;
        int baud_rate_;

    }; // class Interactive

} // namespace iahrs

#endif // IAHRS_SERIAL_INTERACTIVE_BRIDGE_HPP_
