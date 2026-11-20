#pragma once

#include <memory>
#include <optional>
#include <string>
#include <vector>

#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/point.hpp>
#include <geometry_msgs/msg/twist_stamped.hpp>
#include <geometry_msgs/msg/vector3.hpp>
#include <mavros_msgs/msg/estimator_status.hpp>
#include <mavros_msgs/msg/extended_state.hpp>
#include <mavros_msgs/msg/position_target.hpp>
#include <mavros_msgs/msg/state.hpp>
#include <mavros_msgs/srv/command_bool.hpp>
#include <mavros_msgs/srv/command_tol.hpp>
#include <mavros_msgs/srv/set_mode.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <std_msgs/msg/float64.hpp>

class WinchSupervisor : public rclcpp::Node
{
    public:
        template<typename Message>
        struct Received
        {
            std::shared_ptr<const Message> message;
            rclcpp::Time received_at{0, 0, RCL_ROS_TIME};
        };

        struct QuadTelemetry
        {
            Received<mavros_msgs::msg::State> state;
            Received<mavros_msgs::msg::ExtendedState> extended_state;
            Received<mavros_msgs::msg::EstimatorStatus> estimator_status;
            Received<nav_msgs::msg::Odometry> odometry;
            Received<geometry_msgs::msg::TwistStamped> local_velocity;
            Received<std_msgs::msg::Float64> relative_altitude;
        };

        struct TetherTelemetry
        {
            Received<std_msgs::msg::Float64> length;
            Received<std_msgs::msg::Float64> range;
            Received<std_msgs::msg::Float64> extension;
            Received<std_msgs::msg::Float64> payout_rate;
            Received<std_msgs::msg::Float64> tension;
            Received<std_msgs::msg::Float64> azimuth;
            Received<std_msgs::msg::Float64> elevation;
            Received<std_msgs::msg::Float64> drum_load_torque;
            Received<geometry_msgs::msg::Vector3> anchor_force;
        };

        template<typename Service>
        using PendingRequest = std::optional<typename rclcpp::Client<Service>::FutureAndRequestId>;

        WinchSupervisor();

        // Read from callbacks in this node's single-threaded executor.
        const QuadTelemetry &quad_telemetry() const { return quad_; }
        const TetherTelemetry &tether_telemetry() const { return tether_; }
        bool fcu_connected() const;
        bool guided_and_armed() const;

        // MAVROS converts the local ROS ENU values to MAVLink NED.
        bool publish_local_position(const geometry_msgs::msg::Point &position_enu_m);
        bool publish_local_velocity(const geometry_msgs::msg::Vector3 &velocity_enu_mps);
        bool publish_winch_length(double length_m);

        PendingRequest<mavros_msgs::srv::SetMode> request_mode(const std::string &mode);
        PendingRequest<mavros_msgs::srv::CommandBool> request_arm(bool arm);
        PendingRequest<mavros_msgs::srv::CommandTOL> request_takeoff(float relative_altitude_m);
        PendingRequest<mavros_msgs::srv::CommandTOL> request_land();
        void hover_above_drum_at();
        // create an rclcpp Timer for periodically controlling the relative position above the drum
        rclcpp::TimerBase::SharedPtr control_timer_;

    private:
        template<typename Message>
        void subscribe_to(const std::string &topic, Received<Message> &sample);
        bool publish_setpoint(mavros_msgs::msg::PositionTarget &setpoint);

        QuadTelemetry quad_;
        TetherTelemetry tether_;
        // For demonstrations purposes
        geometry_msgs::msg::Vector3 demo_station_position;

        std::vector<rclcpp::SubscriptionBase::SharedPtr> subscriptions_;
        rclcpp::Publisher<mavros_msgs::msg::PositionTarget>::SharedPtr local_setpoint_pub_;
        rclcpp::Publisher<std_msgs::msg::Float64>::SharedPtr winch_length_pub_;
        rclcpp::Publisher<geometry_msgs::msg::Vector3>::SharedPtr relative_position_pub_;
        rclcpp::Publisher<geometry_msgs::msg::Vector3>::SharedPtr transformed_quad_position_pub_;

        rclcpp::Client<mavros_msgs::srv::SetMode>::SharedPtr mode_client_;
        rclcpp::Client<mavros_msgs::srv::CommandBool>::SharedPtr arm_client_;
        rclcpp::Client<mavros_msgs::srv::CommandTOL>::SharedPtr takeoff_client_;
        rclcpp::Client<mavros_msgs::srv::CommandTOL>::SharedPtr land_client_;
};
