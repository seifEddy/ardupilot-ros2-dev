#include "robot_description/winch_supervisor.hpp"

WinchSupervisor::WinchSupervisor() : Node("winch_supervisor")
{
    RCLCPP_INFO(this->get_logger(), "WinchSupervisor node has been started.");
    // Setup the subscriptions
    RCLCPP_INFO(this->get_logger(), "Setting up subscriptions.");
    tether_tension_sub_ = this->create_subscription<std_msgs::msg::Float64>(
        "/gz/tether/tension",
        10, std::bind(&WinchSupervisor::tether_tension_updated, this, std::placeholders::_1)
    );
    anchor_force_sub_ = this->create_subscription<geometry_msgs::msg::Vector3>(
        "/gz/tether/anchor_force",
        10, std::bind(&WinchSupervisor::anchor_force_updated, this, std::placeholders::_1)
    );
}

void WinchSupervisor::tether_tension_updated(const std_msgs::msg::Float64::SharedPtr msg)
{
    RCLCPP_DEBUG(this->get_logger(), "Tether tension received: %f", msg->data);
    tether_tension_ = msg;
}

void WinchSupervisor::anchor_force_updated(const geometry_msgs::msg::Vector3::SharedPtr msg)
{
    RCLCPP_DEBUG(this->get_logger(), "Anchor force received: x=%f, y=%f, z=%f", msg->x, msg->y, msg->z);
    anchor_force_ = msg;
}