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
 * @file    serial_packet.hpp
 * @brief   IAHRS RB-SDA-v1 serial packet warppers and variables
 * @author  Minkyu Kil
 * @date    2025-06-17
 * @version 1.0
 *
 * -------------------------------------------------------------------------------------------------
 */

#ifndef IAHRS_SERIAL_SERIAL_PACKET_HPP_
#define IAHRS_SERIAL_SERIAL_PACKET_HPP_

#include <array>
#include <cmath>

namespace iahrs
{
    /** @brief Data index wrapper */
    enum Packet
    {
        // Linear acceleration
        ACCEL_X = 0,
        ACCEL_Y = 1,
        ACCEL_Z = 2,

        // Gyroscope
        GYRO_X = 3,
        GYRO_Y = 4,
        GYRO_Z = 5,

        // Magnetic field
        MAGNET_X = 6,
        MAGNET_Y = 7,
        MAGNET_Z = 8,

        // Quaternion orientation
        QUAT_W = 9,
        QUAT_X = 10,
        QUAT_Y = 11,
        QUAT_Z = 12,

        // Packet size
        PACKET_SIZE = 13

    }; // enum Packet

    /** @brief Factor to convert degree to radians */
    static constexpr double DEG2RAD {M_PI / 180.0};

    /** @brief Factor to convert radians to degree*/
    static constexpr double RAD2DEG {180.0 / M_PI};

    /** @brief Gravitational acceleration [m/s^2] */
    static constexpr double GRAVITATIONAL_ACCELERATION {9.80665};

    /** @brief Default linear acceleration covariance */
    static constexpr std::array<double, 9>  default_orientation_covariance = {
        0.00022689, 0.0       , 0.0       ,
        0.0       , 0.00019199, 0.0       ,
        0.0       , 0.0       , 0.00010472
    };

    /** @brief Default linear acceleration covariance */
    static constexpr std::array<double, 9> default_acceleration_covariance = {
        0.0064, 0.0   , 0.0   ,
        0.0   , 0.0063, 0.0   ,
        0.0   , 0.0   , 0.0064
    };

    /** @brief Default angular velocity covariance */
    static constexpr std::array<double, 9> default_angular_velocity_covariance = {
        0.0005585, 0.0      , 0.0       ,
        0.0      , 0.0004887, 0.0       ,
        0.0      , 0.0      , 0.00010472
    };

} // namespace iahrs

#endif // IAHRS_SERIAL_SERIAL_PACKET_HPP_