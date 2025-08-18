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
 * @file    serial_bridge.hpp
 * @brief   IAHRS RB-SDA-v1 sync data publisher
 * @author  Minkyu Kil
 * @date    2025-06-17
 * @version 1.0
 *
 * -------------------------------------------------------------------------------------------------
 */

#ifndef IAHRS_SERIAL_SERIAL_BRIDGE_HPP_
#define IAHRS_SERIAL_SERIAL_BRIDGE_HPP_

#include "iahrs_serial/serial_port.hpp"

#include "sensor_msgs/msg/magnetic_field.hpp"
#include "sensor_msgs/msg/imu.hpp"
#include "rclcpp/rclcpp.hpp"

#include <string>

namespace iahrs
{
    class SerialBridge : public rclcpp::Node
    {
    public:

        SerialBridge();

        ~SerialBridge();

    private:

        void timer_callback();

        void initialize_node();

        void declare_parameters();

    private:

        // Serial port
        std::unique_ptr<SerialPort> serial_port_;
        std::string port_path_;
        int baud_rate_;

        // Timers
        rclcpp::TimerBase::SharedPtr timer_;

        // Publishers
        rclcpp::Publisher<sensor_msgs::msg::Imu>::SharedPtr imu_pub_;
        rclcpp::Publisher<sensor_msgs::msg::MagneticField>::SharedPtr magnetic_field_pub_;

        // Sensor data
        sensor_msgs::msg::Imu imu_data_;
        sensor_msgs::msg::MagneticField magnetic_field_data_;

        // Frame ID and topic
        std::string frame_id_;
        std::string imu_topic_;
        std::string magnetic_field_topic_;

        // Parameters
        bool remove_gravitational_acceleration_;
        
    }; // class SerialBridge

} // namespace iahrs

#endif // IAHRS_SERIAL_SERIAL_BRIDGE_HPP_