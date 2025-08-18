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
 * @file    interactive_bridge_node.cpp
 * @brief   IAHRS RB-SDA-v1 interactive interface node
 * @author  Minkyu Kil
 * @date    2025-06-17
 * @version 1.0
 *
 * -------------------------------------------------------------------------------------------------
 */

#include "iahrs_serial/interactive_bridge.hpp"
#include "iahrs_util/log.hpp"
#include <iostream>

int main(int argc, char *argv[])
{
    rclcpp::init(argc, argv);
    std::shared_ptr<iahrs::InteractiveBridge> node;
    try
    {
        node = std::make_shared<iahrs::InteractiveBridge>();

        // Interactive node with terminal
        std::cout << "[IAHRS interactive serial node]" << std::endl;
        std::cout << "Type 'exit' to close interactive ndoe" << std::endl;
        while(true)
        {
            // Command input
            std::string command;
            std::cout << "command : ";
            std::getline(std::cin, command);
        
            // Exit case
            if(command == "exit")
            {
                break;
            }

            // Query
            command += "\n";
            node->query(command.c_str());
        }
    }
    catch(const std::exception &ex)
    {
        IAHRS_ERROR("%s", ex.what());
    }

    rclcpp::shutdown();
    return 0;
}