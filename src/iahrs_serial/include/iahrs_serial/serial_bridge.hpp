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
#include "iahrs_msgs/srv/initialize_orientation.hpp"
#include "iahrs_msgs/srv/set_orientation_zero.hpp"

#include "sensor_msgs/msg/magnetic_field.hpp"
#include "sensor_msgs/msg/imu.hpp"
#include "rclcpp/rclcpp.hpp"

#include <string>

namespace iahrs
{
    /**
     * @brief ROS 2 node that bridges an IMU over a serial link.
     * @details Opens a serial port, reads CSV IMU frames,
     *  converts to SI units, and publishes sensor_msgs::msg::Imu and
     *  sensor_msgs::msg::MagneticField with a shared timestamp.
     */
    class SerialBridge : public rclcpp::Node
    {
    // "SerialBridge" member functions
    public:

        /**
         * @brief Default class contructor
         * @details Initializes the base Node with name "iahrs_serial_bridge".
         */
        SerialBridge();

        /**
         * @brief Default class destructor
         * @details Destroys the Serial node, closing and deallocating the serial port.
         */
        ~SerialBridge();

    private:

        /**
         * @brief Timer callback that receives one synchronous CSV frame from the sensor,
         * parses numeric fields into SI units, and publishes IMU & MagneticField.
         * @details Input CSV layout
         * - Unit conversion:
         *   Accel (g)      → m/s² : * GRAVITATIONAL_ACCELERATION
         *   Gyro  (deg/s)  → rad/s: * DEG2RAD
         *   Magn. (µT)     → T    : * 1e-6
         *   Euler (deg)    → rad  : * DEG2RAD (for quaternion conversion)
         */
        void timer_callback();

        /**
         * @brief Service callback to initialize the orientation of the IMU.
         * @param request Service request (not used in this implementation).
         * @param response Service response containing the result of the operation.
         */
        void initialize_orientation_callback(
            const std::shared_ptr<iahrs_msgs::srv::InitializeOrientation::Request> /*request*/,
            std::shared_ptr<iahrs_msgs::srv::InitializeOrientation::Response> response);

        /**
         * @brief Service callback to set the current orientation as zero reference.
         * @param request Service request (not used in this implementation).
         * @param response Service response containing the result of the operation.
         */
        void set_orientation_zero_callback(
            const std::shared_ptr<iahrs_msgs::srv::SetOrientationZero::Request> /*request*/,
            std::shared_ptr<iahrs_msgs::srv::SetOrientationZero::Response> response);

        /** @brief Initializes timers, publishers, service server, and the serial port. */
        void initialize_node();

        /** @brief Declares and retrieves ROS2 parameters for serial and IAHRS configuration. */
        void declare_parameters();

    // "SerialBridge" member variables
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

        // Services
        rclcpp::Service<iahrs_msgs::srv::InitializeOrientation>::SharedPtr initialize_orientation_srv_;
        rclcpp::Service<iahrs_msgs::srv::SetOrientationZero>::SharedPtr set_orientation_zero_srv_;

        // Sensor data
        sensor_msgs::msg::Imu imu_data_;
        sensor_msgs::msg::MagneticField magnetic_field_data_;

        // Frame ID and topic
        std::string frame_id_;
        std::string imu_topic_;
        std::string magnetic_field_topic_;

        // Parameters
        bool remove_gravitational_acceleration_;

        // Orientation offset
        double roll_offset_deg_;
        double pitch_offset_deg_;
        double yaw_offset_deg_;
        
    }; // class SerialBridge

} // namespace iahrs

#endif // IAHRS_SERIAL_SERIAL_BRIDGE_HPP_