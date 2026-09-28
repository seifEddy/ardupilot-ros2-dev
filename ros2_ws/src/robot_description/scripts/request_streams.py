#!/usr/bin/env python3
"""Request streams after MAVROS connects, and reapply after reconnection.

StreamRate has an empty response: success means the request was dispatched,
not that the autopilot acknowledged it. Send three requests per connection,
three ROS-clock seconds apart, to tolerate UDP loss during startup. This node neither
arms the vehicle nor sends flight setpoints.
"""

import rclpy
from rclpy.clock import Clock, ClockType
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from mavros_msgs.msg import State
from mavros_msgs.srv import StreamRate


class StreamSetup(Node):
    def __init__(self):
        super().__init__("mavros_stream_setup")
        self.stream_id = self.declare_parameter("stream_id", 0).value
        self.stream_rate = self.declare_parameter("stream_rate", 10).value
        if type(self.stream_id) is not int or self.stream_id not in (0, 1, 2, 3, 4, 6, 10, 11, 12):
            raise ValueError("stream_id must be a supported MAVLink stream group")
        if type(self.stream_rate) is not int or not 1 <= self.stream_rate <= 100:
            raise ValueError("stream_rate must be an integer between 1 and 100 Hz")
        self.connected = False
        self.last_clock = None
        self.last_tick = None
        self.last_state = None
        self.future = None
        self.request_started = 0.0
        self.next_request = 0.0
        self.sent = 0
        self.attempts = 0
        self.exhausted_logged = False
        self.client = self.create_client(StreamRate, "set_stream_rate")
        self.subscription = self.create_subscription(
            State, "state", self.on_state, qos_profile_sensor_data
        )
        # Poll with a steady timer so clock resets can always be observed.
        # All deadlines and progress below use the node's ROS clock.
        self.poll_clock = Clock(clock_type=ClockType.STEADY_TIME)
        self.timer = self.create_timer(0.5, self.tick, clock=self.poll_clock)
        self.get_logger().info(f"Waiting for FCU connection and stream service in namespace {self.get_namespace()}")

    def reset_connection(self):
        if self.future is not None:
            self.client.remove_pending_request(self.future)
            self.future.cancel()
            self.future = None
        self.sent = 0
        self.attempts = 0
        self.next_request = 0.0
        self.exhausted_logged = False

    def ros_now(self):
        now = self.get_clock().now().nanoseconds / 1e9
        if self.last_clock is not None and now < self.last_clock:
            self.reset_connection()
            self.connected = False
            self.last_state = None
            self.last_tick = None
            self.get_logger().warning("ROS clock moved backwards; waiting for fresh FCU state")
        self.last_clock = now
        return now

    def on_state(self, state):
        self.last_state = self.ros_now()
        if state.connected != self.connected:
            self.reset_connection()
            self.connected = state.connected
            if self.connected:
                self.get_logger().info("FCU connected; scheduling telemetry requests")

    def tick(self):
        now = self.ros_now()
        # No retries or stale-state decisions before /clock starts or while
        # Gazebo is paused. A steady poll does not advance simulation time.
        if self.get_parameter("use_sim_time").value and now <= 0:
            return
        if self.last_tick == now:
            return
        self.last_tick = now
        # Also handle MAVROS disappearing without publishing disconnected state.
        if self.connected and self.last_state is not None and now - self.last_state > 15:
            self.connected = False
            self.reset_connection()
            self.get_logger().warning("MAVROS state stream stale; waiting for reconnection")
        if not self.connected:
            return

        if self.future is not None:
            if self.future.done():
                try:
                    self.future.result()
                except Exception as exc:
                    self.get_logger().warning(f"Stream service failed: {exc}")
                else:
                    self.sent += 1
                    self.get_logger().info(
                        f"Telemetry request sent ({self.sent}/3): "
                        f"stream_id={self.stream_id}, rate={self.stream_rate} Hz"
                    )
                self.future = None
                self.next_request = now + 3
            elif now - self.request_started > 5:
                self.client.remove_pending_request(self.future)
                self.future.cancel()
                self.future = None
                self.next_request = now + 3
                self.get_logger().warning("Stream service timed out; will retry")
            return

        if self.sent >= 3:
            return
        if self.attempts >= 6:
            if not self.exhausted_logged:
                self.get_logger().error(
                    "Stream setup failed repeatedly. Check MAVROS; restart this launch to retry."
                )
                self.exhausted_logged = True
            return
        if now < self.next_request or not self.client.service_is_ready():
            return
        request = StreamRate.Request()
        request.stream_id = self.stream_id
        request.message_rate = self.stream_rate
        request.on_off = True
        self.attempts += 1
        self.request_started = now
        self.future = self.client.call_async(request)


def main(args=None):
    rclpy.init(args=args)
    try:
        node = StreamSetup()
    except Exception:
        rclpy.shutdown()
        raise
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()