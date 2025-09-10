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
 * @file    sensor_monitor.hpp
 * @brief   IAHRS RB-SDA-v1 Qt5 sensor monitor
 * @author  Minkyu Kil
 * @date    2025-06-17
 * @version 1.0
 *
 * -------------------------------------------------------------------------------------------------
 */

#ifndef IAHRS_GUI_SENSOR_MONITOR_HPP_
#define IAHRS_GUI_SENSOR_MONITOR_HPP_

#include "iahrs_msgs/srv/initialize_orientation.hpp"
#include "iahrs_msgs/srv/set_orientation_zero.hpp"

#include "sensor_msgs/msg/imu.hpp"
#include "sensor_msgs/msg/magnetic_field.hpp"
#include "rclcpp/rclcpp.hpp"

#include <QDockWidget>

namespace Ui
{
    // Forwarding Qt5 sensor monitor widget class
    class SensorMonitorDockWidget;

} // namesapce Ui

namespace iahrs
{
    class SensorMonitor : public QDockWidget, public rclcpp::Node
    {
    // Qt5 Object macro
    Q_OBJECT

    // "SensorMonitor" member functions
    public:

        explicit SensorMonitor(QWidget *parent = nullptr);

        ~SensorMonitor() override;

    private:

        void imu_callback(const sensor_msgs::msg::Imu::SharedPtr msg);

        void magnetic_field_callback(const sensor_msgs::msg::MagneticField::SharedPtr msg);

        void request_initialize_orientation();

        void request_set_orientation_zero();

        void read_serial_bridge_parameters();

        void initialize_node();

    // "SensorMonitor" member variables
    private:

        // Qt5 sensor monitor widget
        std::unique_ptr<Ui::SensorMonitorDockWidget> sensor_monitor_widget_;

        // Subscribers
        rclcpp::Subscription<sensor_msgs::msg::Imu>::SharedPtr imu_sub_;
        rclcpp::Subscription<sensor_msgs::msg::MagneticField>::SharedPtr magnetic_field_sub_;

        // Service clients
        rclcpp::Client<iahrs_msgs::srv::InitializeOrientation>::SharedPtr initialize_orientation_client_;
        rclcpp::Client<iahrs_msgs::srv::SetOrientationZero>::SharedPtr set_orientation_zero_client_;

        // Parameter clients
        std::shared_ptr<rclcpp::SyncParametersClient> parameter_client_;

    }; // SensorMonitor

} // namespace iahrs

#endif // IAHRS_GUI_SENSOR_MONITOR_HPP_