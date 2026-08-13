#include "robot_description/robot_controller_node.hpp"

namespace robot_description
{

RobotControllerNode::RobotControllerNode(const rclcpp::NodeOptions & options)
: Node("robot_controller_node", options)
{
  // Publish Twist commands consumed by diff_drive_controller.
  // The controller subscribes to ~/cmd_vel (use_stamped_vel: false → plain Twist).
  cmd_vel_pub_ = this->create_publisher<geometry_msgs::msg::Twist>(
    "/diff_drive_controller/cmd_vel", 10);

  // Subscribe to joint states published by joint_state_broadcaster.
  joint_state_sub_ = this->create_subscription<sensor_msgs::msg::JointState>(
    "/joint_states", 10,
    std::bind(&RobotControllerNode::onJointState, this, std::placeholders::_1));

  // 10 Hz control loop
  control_timer_ = this->create_wall_timer(
    std::chrono::milliseconds(100),
    std::bind(&RobotControllerNode::controlLoop, this));

  RCLCPP_INFO(get_logger(), "RobotControllerNode started — publishing to /diff_drive_controller/cmd_vel");
}

void RobotControllerNode::onJointState(const sensor_msgs::msg::JointState::SharedPtr msg)
{
  for (std::size_t i = 0; i < msg->name.size(); ++i) {
    if (i >= msg->velocity.size()) {
      break;
    }
    if (msg->name[i] == "left_wheel_joint") {
      left_wheel_vel_ = msg->velocity[i];
    } else if (msg->name[i] == "right_wheel_joint") {
      right_wheel_vel_ = msg->velocity[i];
    }
  }
}

void RobotControllerNode::controlLoop()
{
  // Each phase is 20 cycles × 100 ms = 2 s
  constexpr int phase_len = 20;
  ++cycle_;

  auto twist = geometry_msgs::msg::Twist{};

  if (cycle_ <= phase_len) {
    // Phase 1 – drive forward
    twist.linear.x  = 0.5;
    twist.angular.z = 0.0;
  } else if (cycle_ <= phase_len * 2) {
    // Phase 2 – rotate in place
    twist.linear.x  = 0.0;
    twist.angular.z = 0.8;
  } else if (cycle_ <= phase_len * 3) {
    // Phase 3 – drive forward again
    twist.linear.x  = 0.5;
    twist.angular.z = 0.0;
  } else {
    // Phase 4 – stop
    twist.linear.x  = 0.0;
    twist.angular.z = 0.0;
  }

  cmd_vel_pub_->publish(twist);

  RCLCPP_DEBUG(
    get_logger(),
    "[cycle %3d] cmd_vel: lin=%.2f ang=%.2f | wheel_vel: L=%.3f R=%.3f",
    cycle_, twist.linear.x, twist.angular.z,
    left_wheel_vel_, right_wheel_vel_);
}

}  // namespace robot_description

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<robot_description::RobotControllerNode>());
  rclcpp::shutdown();
  return 0;
}
