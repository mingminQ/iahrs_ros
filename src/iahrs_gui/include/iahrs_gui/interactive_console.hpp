#ifndef IAHRS_GUI_INTERACTIVE_CONSOLE_HPP_
#define IAHRS_GUI_INTERACTIVE_CONSOLE_HPP_

#include "iahrs_serial/serial_port.hpp"

#include "rclcpp/rclcpp.hpp"

namespace iahrs
{
    class InteractiveConsole : rclcpp::Node
    {
    // "InteractiveConsole" memeber functions
    public:
        
        InteractiveConsole();

        ~InteractiveConsole() override;

    // "InteractoveConsole" member variables
    private:



    }; // class InteractiveConsole

} // namespace iahrs

#endif // IAHRS_GUI_INTERACTIVE_CONSOLE_HPP_