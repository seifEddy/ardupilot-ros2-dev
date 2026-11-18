#include "robot_description/winch_supervisor.hpp"
#include <rclcpp/rclcpp.hpp>

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<WinchSupervisor>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}