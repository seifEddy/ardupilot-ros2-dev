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
    this->declare_parameter<double>("target_altitude_z", 15.0);

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
    relative_position_pub_ = this->create_publisher<geometry_msgs::msg::Vector3>(
        tether + "/relative_position", 1);
    transformed_quad_position_pub_ = this->create_publisher<geometry_msgs::msg::Vector3>(
        tether + "/transformed_quad_position", 1);

    mode_client_ = this->create_client<mavros_msgs::srv::SetMode>(mavros + "/set_mode");
    arm_client_ = this->create_client<mavros_msgs::srv::CommandBool>(mavros + "/cmd/arming");
    takeoff_client_ = this->create_client<mavros_msgs::srv::CommandTOL>(mavros + "/cmd/takeoff");
    land_client_ = this->create_client<mavros_msgs::srv::CommandTOL>(mavros + "/cmd/land");

    // Attach the control timer to this->hover_above_drum_at
    control_timer_ = this->create_wall_timer(
        std::chrono::milliseconds(100), std::bind(&WinchSupervisor::hover_above_drum_at, this)
    );

    // Initialize demo station position
    demo_station_position.x = -1.4;
    demo_station_position.y = 0.0;
    demo_station_position.z = 0.0;

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

void WinchSupervisor::hover_above_drum_at()
{
    // Use the current tether length, and azimuth/elevation angles to calculate the relative NED velocity.
    // This velocity can then be used to publish a local velocity setpoint to:
    //   -  Regulate the quad's XY position by minimizing the XY position error.
    //   -  Control its altitude relative to the drum.
    const double relative_altitude_m = this->get_parameter("target_altitude_z").as_double();
    RCLCPP_INFO(this->get_logger(), "Hovering above drum at relative altitude: %f", relative_altitude_m);
    if (!std::isfinite(relative_altitude_m) || relative_altitude_m <= 0.5)
    {
        return;
    }
    if (!this->tether_.length.message || !this->tether_.anchor_force.message)
    {
        return;
    }
    // arm the quad if not already armed, and ensure it is in guided mode before attempting to control its position.
    if (!this->guided_and_armed())
    {
        RCLCPP_INFO(this->get_logger(), "Requesting GUIDED mode and arming.");
        this->request_mode("GUIDED");
        this->request_arm(true);
        return;
    }
    // when it is guided and armed, takeoff until reaching a third of the desired relative altitude.
    if (tether_.length.message->data < relative_altitude_m / 3.0)
    {
        RCLCPP_INFO(this->get_logger(), "Taking off to a third of the desired relative altitude.");
        this->request_takeoff(10);
        return;
    }
    geometry_msgs::msg::Vector3 velocity_pid_control;
    // Comute xy from rho theta (polar coordinates)
    if (!tether_.range.message ||
        !tether_.azimuth.message ||
        !tether_.elevation.message)
    {
        return;
    }

    const double range = tether_.range.message->data;
    const double azimuth = tether_.azimuth.message->data;
    const double elevation = tether_.elevation.message->data;

    if (!std::isfinite(range) || range < 0.0 ||
        !std::isfinite(azimuth) || !std::isfinite(elevation))
    {
        return;
    }

    geometry_msgs::msg::Vector3 relative_position;
    relative_position.x = range * std::cos(elevation) * std::cos(azimuth);
    relative_position.y = range * std::cos(elevation) * std::sin(azimuth);
    relative_position.z = range * std::sin(elevation);
    relative_position_pub_->publish(relative_position);
    auto transformed_quad_position = geometry_msgs::msg::Vector3();
    transformed_quad_position.x = relative_position.x + this->demo_station_position.x;
    transformed_quad_position.y = relative_position.y + this->demo_station_position.y;
    transformed_quad_position.z = relative_position.z + this->demo_station_position.z;
    transformed_quad_position_pub_->publish(transformed_quad_position);
    double x_error = -relative_position.x;
    double y_error = -relative_position.y;
    double z_error = relative_altitude_m - relative_position.z;
    // Apply a simple proportional gain for altitude control
    const double Kp_z = 0.2, Kp_xy = 0.25;
    velocity_pid_control.x = x_error * Kp_xy;
    velocity_pid_control.y = y_error * Kp_xy;
    velocity_pid_control.z = z_error * Kp_z;
    RCLCPP_INFO(this->get_logger(), "X error: %f, Y error: %f, Altitude error: %f", x_error, y_error, z_error);
    this->publish_local_velocity(velocity_pid_control);
}
