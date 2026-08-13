#ifndef SAMPLE_ROBOT_DESCRIPTION__ROBOT_CONTROLLER_NODE_HPP_
#define SAMPLE_ROBOT_DESCRIPTION__ROBOT_CONTROLLER_NODE_HPP_

#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/twist.hpp>
#include <sensor_msgs/msg/joint_state.hpp>

namespace robot_description
{

/**
 * Demo node that:
 *  - subscribes to /joint_states to read wheel velocities from ros2_control
 *  - publishes Twist commands to the diff_drive_controller
 *
 * Sequence: forward (2 s) → rotate (2 s) → forward (2 s) → stop
 */
class RobotControllerNode : public rclcpp::Node
{
public:
  explicit RobotControllerNode(const rclcpp::NodeOptions & options = rclcpp::NodeOptions());

private:
  void onJointState(const sensor_msgs::msg::JointState::SharedPtr msg);
  void controlLoop();

  rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr cmd_vel_pub_;
  rclcpp::Subscription<sensor_msgs::msg::JointState>::SharedPtr joint_state_sub_;
  rclcpp::TimerBase::SharedPtr control_timer_;

  double left_wheel_vel_{0.0};
  double right_wheel_vel_{0.0};
  int    cycle_{0};
};

}  // namespace robot_description

#endif  // SAMPLE_ROBOT_DESCRIPTION__ROBOT_CONTROLLER_NODE_HPP_
