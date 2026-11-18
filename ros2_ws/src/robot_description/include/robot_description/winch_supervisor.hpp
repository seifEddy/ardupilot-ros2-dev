#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/float64.hpp>
#include <geometry_msgs/msg/vector3.hpp>

class WinchSupervisor : public rclcpp::Node
{
    public:
        WinchSupervisor();
        void tether_tension_updated(const std_msgs::msg::Float64::SharedPtr msg);
        void anchor_force_updated(const geometry_msgs::msg::Vector3::SharedPtr msg);
    private:
        rclcpp::Subscription<std_msgs::msg::Float64>::SharedPtr tether_tension_sub_;
        rclcpp::Subscription<geometry_msgs::msg::Vector3>::SharedPtr anchor_force_sub_;

        std_msgs::msg::Float64::SharedPtr tether_tension_;
        geometry_msgs::msg::Vector3::SharedPtr anchor_force_;
};