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
 * @file    sensor_monitor.cpp
 * @brief   IAHRS RB-SDA-v1 Qt5 sensor monitor
 * @author  Minkyu Kil
 * @date    2025-06-17
 * @version 1.0
 *
 * -------------------------------------------------------------------------------------------------
 */

#include "iahrs_gui/sensor_monitor.hpp"
#include "iahrs_util/log.hpp"
#include "ui_sensor_monitor.h"

#include "tf2/LinearMath/Quaternion.hpp"
#include "tf2/LinearMath/Matrix3x3.hpp"

#include <QSignalBlocker>
#include <QMessageBox>
#include <QTimer>

#include <functional>
#include <algorithm>
#include <chrono>
#include <cmath>

iahrs::SensorMonitor::SensorMonitor(QWidget *parent)
  : QDockWidget(parent),
    Node("iahrs_sensor_monitor")
{
    initialize_node();
}

iahrs::SensorMonitor::~SensorMonitor()
{
}

void iahrs::SensorMonitor::imu_callback(const sensor_msgs::msg::Imu::SharedPtr msg)
{
    tf2::Quaternion quaternion(
        msg->orientation.x, msg->orientation.y, msg->orientation.z, msg->orientation.w);
    double roll, pitch, yaw;
    tf2::Matrix3x3(quaternion).getRPY(roll, pitch, yaw);

    const double acceleration_x = msg->linear_acceleration.x;
    const double acceleration_y = msg->linear_acceleration.y;
    const double acceleration_z = msg->linear_acceleration.z;

    const double angular_velocity_x = msg->angular_velocity.x;
    const double angular_velocity_y = msg->angular_velocity.y;
    const double angular_velocity_z = msg->angular_velocity.z;

    const QString roll_text  = QString::number(roll  * 180.0 / M_PI, 'f', 3) + " deg";
    const QString pitch_text = QString::number(pitch * 180.0 / M_PI, 'f', 3) + " deg";
    const QString yaw_text   = QString::number(yaw   * 180.0 / M_PI, 'f', 3) + " deg";

    const QString acceleration_x_text = QString::number(acceleration_x, 'f', 3) + " m/s^2";
    const QString acceleration_y_text = QString::number(acceleration_y, 'f', 3) + " m/s^2";
    const QString acceleration_z_text = QString::number(acceleration_z, 'f', 3) + " m/s^2";

    const QString angular_velocity_x_text = QString::number(angular_velocity_x, 'f', 3) + " rad/s";
    const QString angular_velocity_y_text = QString::number(angular_velocity_y, 'f', 3) + " rad/s";
    const QString angular_velocity_z_text = QString::number(angular_velocity_z, 'f', 3) + " rad/s";

    QMetaObject::invokeMethod(this,
        [this,
            roll_text, pitch_text, yaw_text,
            acceleration_x_text, acceleration_y_text, acceleration_z_text,
            angular_velocity_x_text, angular_velocity_y_text, angular_velocity_z_text]()
        {
            sensor_monitor_widget_->orientation_roll_text_box->setPlainText(roll_text);
            sensor_monitor_widget_->orientation_pitch_text_box->setPlainText(pitch_text);
            sensor_monitor_widget_->orientation_yaw_text_box->setPlainText(yaw_text);

            sensor_monitor_widget_->acceleration_x_text_box->setPlainText(acceleration_x_text);
            sensor_monitor_widget_->acceleration_y_text_box->setPlainText(acceleration_y_text);
            sensor_monitor_widget_->acceleration_z_text_box->setPlainText(acceleration_z_text);

            sensor_monitor_widget_->angular_velocity_x_text_box->setPlainText(angular_velocity_x_text);
            sensor_monitor_widget_->angular_velocity_y_text_box->setPlainText(angular_velocity_y_text);
            sensor_monitor_widget_->angular_velocity_z_text_box->setPlainText(angular_velocity_z_text);
        }, Qt::QueuedConnection
    );
}

void iahrs::SensorMonitor::magnetic_field_callback(const sensor_msgs::msg::MagneticField::SharedPtr msg)
{
    const double magnetic_field_x = msg->magnetic_field.x * 1e6;
    const double magnetic_field_y = msg->magnetic_field.y * 1e6;
    const double magnetic_field_z = msg->magnetic_field.z * 1e6;

    const QString magnetic_field_x_text = QString::number(magnetic_field_x, 'f', 3) + " uT";
    const QString magnetic_field_y_text = QString::number(magnetic_field_y, 'f', 3) + " uT";
    const QString magnetic_field_z_text = QString::number(magnetic_field_z, 'f', 3) + " uT";

    QMetaObject::invokeMethod(this,
        [this, magnetic_field_x_text, magnetic_field_y_text, magnetic_field_z_text]()
        {
            sensor_monitor_widget_->magnetic_field_x_text_box->setPlainText(magnetic_field_x_text);
            sensor_monitor_widget_->magnetic_field_y_text_box->setPlainText(magnetic_field_y_text);
            sensor_monitor_widget_->magnetic_field_z_text_box->setPlainText(magnetic_field_z_text);
        }, Qt::QueuedConnection
    );
}

void iahrs::SensorMonitor::request_initialize_orientation()
{
    if(!initialize_orientation_client_)
    {
        IAHRS_ERROR("SensorMonitor::request_initialize_orientation() Service client is not initialized.");
        QMessageBox::warning(this, "Error", "Service client is not initialized.");
        return;
    }

    if(!initialize_orientation_client_->wait_for_service(std::chrono::milliseconds(200)))
    {
        IAHRS_ERROR("SensorMonitor::request_initialize_orientation() \"/iahrs/initialize_orientation\" service is not available.");
        QMessageBox::warning(this, "Error", "\"/iahrs/initialize_orientation\" service is not available.");
        return;
    }

    // Lock button
    sensor_monitor_widget_->initialize_orientation_request_button->setEnabled(false);

    // Initialize orientation request
    auto initialize_orientation_request = std::make_shared<iahrs_msgs::srv::InitializeOrientation::Request>();

    // Send request
    initialize_orientation_client_->async_send_request(initialize_orientation_request,
        [this](rclcpp::Client<iahrs_msgs::srv::InitializeOrientation>::SharedFuture future)
        {
            auto response = future.get();
            if (response && response->result == false)
            {
                IAHRS_ERROR("SensorMonitor::request_initialize_orientation() Failed to initialize orientation.");
                QMessageBox::warning(this, "ERROR", "Failed to initialize orientation.");
            }

            // Enable button
            sensor_monitor_widget_->initialize_orientation_request_button->setEnabled(true);
        }
    );
}

void iahrs::SensorMonitor::request_set_orientation_zero()
{
    if(!set_orientation_zero_client_)
    {
        IAHRS_ERROR("SensorMonitor::request_set_orientation_zero() Service client is not initialized.");
        QMessageBox::warning(this, "Error", "Service client is not initialized.");
        return;
    }

    if(!set_orientation_zero_client_->wait_for_service(std::chrono::milliseconds(200)))
    {
        IAHRS_ERROR("SensorMonitor::request_set_orientation_zero() \"/iahrs/set_orientation_zero\" service is not available.");
        QMessageBox::warning(this, "Error", "\"/iahrs/set_orientation_zero\" service is not available.");
        return;
    }

    // Lock button
    sensor_monitor_widget_->set_orientation_zero_request_button->setEnabled(false);

    // Set orientation zero request
    auto set_orientation_zero_request = std::make_shared<iahrs_msgs::srv::SetOrientationZero::Request>();

    // Send request
    set_orientation_zero_client_->async_send_request(set_orientation_zero_request,
        [this](rclcpp::Client<iahrs_msgs::srv::SetOrientationZero>::SharedFuture future)
        {
            auto response = future.get();
            if (response && response->result == false)
            {
                IAHRS_ERROR("SensorMonitor::request_set_orientation_zero() Failed to set orientation zero.");
                QMessageBox::warning(this, "ERROR", "Failed to set orientation zero.");
            }

            // Enable button
            sensor_monitor_widget_->set_orientation_zero_request_button->setEnabled(true);
        }
    );
}

void iahrs::SensorMonitor::read_serial_bridge_parameters()
{
    // Read serial bridge parameter and display
    if(parameter_client_->wait_for_service(std::chrono::seconds(10)))
    {
        std::string port_path = parameter_client_->get_parameter<std::string>("port_path");
        sensor_monitor_widget_->serial_port_text_box->setPlainText(QString::fromStdString(port_path));

        int baud_rate = parameter_client_->get_parameter<int>("baud_rate");
        sensor_monitor_widget_->baud_rate_text_box->setPlainText(QString::number(baud_rate));

        bool remove_gravitational_acceleration = parameter_client_->get_parameter<bool>("remove_gravitational_acceleration");
        sensor_monitor_widget_->remove_gravitional_acceleration_text_box->setPlainText(
            remove_gravitational_acceleration ? "True" : "False"
        );

        double roll_offset_deg = parameter_client_->get_parameter<double>("roll_offset_deg");
        sensor_monitor_widget_->roll_offset_text_box->setPlainText(QString::number(roll_offset_deg, 'f', 3) + " deg");

        double pitch_offset_deg = parameter_client_->get_parameter<double>("pitch_offset_deg");
        sensor_monitor_widget_->pitch_offset_text_box->setPlainText(QString::number(pitch_offset_deg, 'f', 3) + " deg");

        double yaw_offset_deg = parameter_client_->get_parameter<double>("yaw_offset_deg");
        sensor_monitor_widget_->yaw_offset_text_box->setPlainText(QString::number(yaw_offset_deg, 'f', 3) + " deg");
    }
}

void iahrs::SensorMonitor::initialize_node()
{
    // Subscribers
    imu_sub_ = this->create_subscription<sensor_msgs::msg::Imu>(
        "/iahrs/imu", 
         rclcpp::QoS(rclcpp::KeepLast(1)).best_effort().durability_volatile(),
        std::bind(&SensorMonitor::imu_callback, this, std::placeholders::_1)
    );

    magnetic_field_sub_ = this->create_subscription<sensor_msgs::msg::MagneticField>(
        "/iahrs/magnetic_field", 
         rclcpp::QoS(rclcpp::KeepLast(1)).best_effort().durability_volatile(),
        std::bind(&SensorMonitor::magnetic_field_callback, this, std::placeholders::_1)
    );

    // Service clients
    initialize_orientation_client_ = this->create_client<iahrs_msgs::srv::InitializeOrientation>(
        "/iahrs/initialize_orientation"
    );

    set_orientation_zero_client_ = this->create_client<iahrs_msgs::srv::SetOrientationZero>(
        "/iahrs/set_orientation_zero"
    );

    // Parameter clients
    parameter_client_ = std::make_shared<rclcpp::SyncParametersClient>(
        this, "/iahrs_serial_bridge"
    );

    // Qt5 GUI sensor monitor
    sensor_monitor_widget_ = std::make_unique<Ui::SensorMonitorDockWidget>();
    sensor_monitor_widget_->setupUi(this);

    // Service button bindings
    QObject::connect(
        sensor_monitor_widget_->initialize_orientation_request_button, &QPushButton::clicked,
        this, &SensorMonitor::request_initialize_orientation
    );

    QObject::connect(
        sensor_monitor_widget_->set_orientation_zero_request_button, &QPushButton::clicked,
        this, &SensorMonitor::request_set_orientation_zero
    );

    // Read serial bridge parameters and display
    read_serial_bridge_parameters();
}
