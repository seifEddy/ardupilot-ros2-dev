#include "robot_description/winch_supervisor.hpp"

#include <chrono>
#include <cmath>
#include <utility>

namespace
{
using PositionTarget = mavros_msgs::msg::PositionTarget;
constexpr auto ignore_acceleration_and_yaw =
    PositionTarget::IGNORE_AFX | PositionTarget::IGNORE_AFY | PositionTarget::IGNORE_AFZ |
    PositionTarget::IGNORE_YAW | PositionTarget::IGNORE_YAW_RATE;

template<typename Service>
WinchSupervisor::PendingRequest<Service> send_request(
    const std::shared_ptr<rclcpp::Client<Service>> &client,
    const std::shared_ptr<typename Service::Request> &request)
{
    // Clear old unanswered requests when a new command is attempted.
    client->prune_requests_older_than(std::chrono::system_clock::now() - std::chrono::seconds(10));
    if (!client->service_is_ready())
    {
        return std::nullopt;
    }
    return client->async_send_request(request);
}
}

WinchSupervisor::WinchSupervisor() : Node("winch_supervisor")
{
    const auto mavros = this->declare_parameter<std::string>("mavros_namespace", "/mavros");
    const auto tether = this->declare_parameter<std::string>("tether_namespace", "/gz/tether");

    this->subscribe_to(mavros + "/state", quad_.state);
    this->subscribe_to(mavros + "/extended_state", quad_.extended_state);
    this->subscribe_to(mavros + "/estimator_status", quad_.estimator_status);
    this->subscribe_to(mavros + "/local_position/odom", quad_.odometry);
    this->subscribe_to(mavros + "/local_position/velocity_local", quad_.local_velocity);
    this->subscribe_to(mavros + "/global_position/rel_alt", quad_.relative_altitude);

    this->subscribe_to(tether + "/length", tether_.length);
    this->subscribe_to(tether + "/range", tether_.range);
    this->subscribe_to(tether + "/extension", tether_.extension);
    this->subscribe_to(tether + "/payout_rate", tether_.payout_rate);
    this->subscribe_to(tether + "/tension", tether_.tension);
    this->subscribe_to(tether + "/azimuth", tether_.azimuth);
    this->subscribe_to(tether + "/elevation", tether_.elevation);
    this->subscribe_to(tether + "/drum_load_torque", tether_.drum_load_torque);
    this->subscribe_to(tether + "/anchor_force", tether_.anchor_force);

    local_setpoint_pub_ = this->create_publisher<PositionTarget>(
        mavros + "/setpoint_raw/local", 1);
    winch_length_pub_ = this->create_publisher<std_msgs::msg::Float64>(
        tether + "/cmd_length", 1);

    mode_client_ = this->create_client<mavros_msgs::srv::SetMode>(mavros + "/set_mode");
    arm_client_ = this->create_client<mavros_msgs::srv::CommandBool>(mavros + "/cmd/arming");
    takeoff_client_ = this->create_client<mavros_msgs::srv::CommandTOL>(mavros + "/cmd/takeoff");
    land_client_ = this->create_client<mavros_msgs::srv::CommandTOL>(mavros + "/cmd/land");

    RCLCPP_INFO(this->get_logger(), "WinchSupervisor monitoring %s and %s.",
        mavros.c_str(), tether.c_str());
}

template<typename Message>
void WinchSupervisor::subscribe_to(const std::string &topic, Received<Message> &sample)
{
    subscriptions_.push_back(this->create_subscription<Message>(
        topic, rclcpp::SensorDataQoS(),
        [this, &sample](std::shared_ptr<const Message> message)
        {
            sample.message = std::move(message);
            sample.received_at = this->now();
        }));
}

bool WinchSupervisor::fcu_connected() const
{
    const auto &state = quad_.state;
    if (!state.message || !state.message->connected)
    {
        return false;
    }
    const auto age = this->now() - state.received_at;
    return age.nanoseconds() >= 0 && age <= rclcpp::Duration::from_seconds(3.0);
}

bool WinchSupervisor::guided_and_armed() const
{
    return this->fcu_connected() && quad_.state.message->armed &&
        quad_.state.message->mode == mavros_msgs::msg::State::MODE_APM_COPTER_GUIDED;
}

bool WinchSupervisor::publish_local_position(const geometry_msgs::msg::Point &position_enu_m)
{
    if (!std::isfinite(position_enu_m.x) || !std::isfinite(position_enu_m.y) ||
        !std::isfinite(position_enu_m.z))
    {
        return false;
    }
    PositionTarget setpoint;
    setpoint.type_mask = ignore_acceleration_and_yaw |
        PositionTarget::IGNORE_VX | PositionTarget::IGNORE_VY | PositionTarget::IGNORE_VZ;
    setpoint.position = position_enu_m;
    return this->publish_setpoint(setpoint);
}

bool WinchSupervisor::publish_local_velocity(const geometry_msgs::msg::Vector3 &velocity_enu_mps)
{
    if (!std::isfinite(velocity_enu_mps.x) || !std::isfinite(velocity_enu_mps.y) ||
        !std::isfinite(velocity_enu_mps.z))
    {
        return false;
    }
    PositionTarget setpoint;
    setpoint.type_mask = ignore_acceleration_and_yaw |
        PositionTarget::IGNORE_PX | PositionTarget::IGNORE_PY | PositionTarget::IGNORE_PZ;
    setpoint.velocity = velocity_enu_mps;
    return this->publish_setpoint(setpoint);
}

bool WinchSupervisor::publish_setpoint(PositionTarget &setpoint)
{
    if (!this->guided_and_armed() || local_setpoint_pub_->get_subscription_count() == 0)
    {
        return false;
    }
    setpoint.header.stamp = this->now();
    setpoint.coordinate_frame = PositionTarget::FRAME_LOCAL_NED;
    local_setpoint_pub_->publish(setpoint);
    return true;
}

bool WinchSupervisor::publish_winch_length(double length_m)
{
    if (!std::isfinite(length_m) || length_m <= 0.0 ||
        winch_length_pub_->get_subscription_count() == 0)
    {
        return false;
    }
    std_msgs::msg::Float64 command;
    command.data = length_m;
    winch_length_pub_->publish(command);
    return true;
}

WinchSupervisor::PendingRequest<mavros_msgs::srv::SetMode>
WinchSupervisor::request_mode(const std::string &mode)
{
    if (!this->fcu_connected() || mode.empty())
    {
        return std::nullopt;
    }
    auto request = std::make_shared<mavros_msgs::srv::SetMode::Request>();
    request->custom_mode = mode;
    return send_request(mode_client_, request);
}

WinchSupervisor::PendingRequest<mavros_msgs::srv::CommandBool>
WinchSupervisor::request_arm(bool arm)
{
    if (!this->fcu_connected())
    {
        return std::nullopt;
    }
    auto request = std::make_shared<mavros_msgs::srv::CommandBool::Request>();
    request->value = arm;
    return send_request(arm_client_, request);
}

WinchSupervisor::PendingRequest<mavros_msgs::srv::CommandTOL>
WinchSupervisor::request_takeoff(float relative_altitude_m)
{
    if (!this->guided_and_armed() || !std::isfinite(relative_altitude_m) ||
        relative_altitude_m <= 0.0F)
    {
        return std::nullopt;
    }
    auto request = std::make_shared<mavros_msgs::srv::CommandTOL::Request>();
    request->altitude = relative_altitude_m;
    return send_request(takeoff_client_, request);
}

WinchSupervisor::PendingRequest<mavros_msgs::srv::CommandTOL>
WinchSupervisor::request_land()
{
    if (!this->fcu_connected())
    {
        return std::nullopt;
    }
    auto request = std::make_shared<mavros_msgs::srv::CommandTOL::Request>();
    return send_request(land_client_, request);
}
