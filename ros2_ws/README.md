# ROS 2 workspace

`robot_description` provides three entry points for the drone workflow. The combined launch supervises SITL, Gazebo, MAVROS, and the Gazebo bridge. The MAVROS-only launch is also the entry point for a physical flight controller. The bridge-only launch connects to an existing Gazebo instance. The differential-drive example remains available through `spawn_robot.launch.py` and requires its own controller dependencies.

## Build

Use [ROS 2 Humble](https://docs.ros.org/en/humble/Installation/Ubuntu-Install-Debs.html) and [the Harmonic build of ros_gz for Humble](https://gazebosim.org/docs/harmonic/ros_installation/#gazebo-harmonic-with-ros-2-humble). Install the Harmonic package from the Gazebo packages repository. MAVROS also needs the GeographicLib geoid dataset.

```bash
sudo apt install ros-humble-ros-gzharmonic ros-humble-mavros ros-humble-mavros-extras
sudo /opt/ros/humble/lib/mavros/install_geographiclib_datasets.sh
source /opt/ros/humble/setup.bash
cd ros2_ws
colcon build --packages-select robot_description
source install/setup.bash
```

If building the differential-drive example, also install its controller dependencies from `src/robot_description/package.xml` using versions compatible with Harmonic. Humble's standard Fortress `ros_gz` packages conflict with the Harmonic packages.

## Run the combined simulation

```bash
ros2 launch robot_description copter_sim.launch.py mode:=gui
```

The default is `mode:=headless world:=quad_x`. Set `world:=quad_x_tether` for the tether station or `world:=iris` for the original Iris runway. If the ROS install is outside the checkout, pass `sitl_script:=/absolute/path/to/ardupilot_ws/start_copter_sim.sh`. Run a single stack per SITL endpoint.

The launcher starts the existing shell script as a managed process and includes both ROS launch files. The Gazebo bridge publishes the world clock once on ROS `/clock`. MAVROS and its telemetry requester use simulation time. The requester waits for a connected FCU and retries stream requests using the ROS clock. It reapplies them after a reconnect and waits for the clock when Gazebo is paused or reset.

Check connectivity and data in a second sourced ROS terminal.

```bash
ros2 topic echo /mavros/state --once
ros2 topic info /clock
ros2 topic echo /gz/imu --once
ros2 topic echo /gz/camera/rear_depth/camera_info --once
```

## Bridge an existing Gazebo instance

```bash
ros2 launch robot_description gazebo_bridge.launch.py world:=quad_x
```

Use the same `world` profile and `GZ_PARTITION` as the running Gazebo instance. The bridge forwards supported Gazebo publications to ROS. Simulation telemetry uses `/gz`, while the standard ROS clock uses `/clock`.

- Both worlds expose poses, dynamic poses, joint states, IMU, the front RGB image and camera calibration, gimbal command telemetry, and Gazebo resource paths.
- Quad X also exposes `/gz/navsat`, `/gz/air_pressure`, `/gz/magnetometer`, and the rear depth image, calibration, and point cloud under `/gz/camera/rear_depth/`.
- The tether world also publishes winch and cable measurements under `/gz/tether/` and accepts `/gz/tether/cmd_length` in metres. See the [tether model](../ardupilot_ws/gz_ws/src/ardupilot_gazebo/docs/quad_x_tether.md).
- The Iris world publishes its reference NavSat under `/gz/world/reference_navsat`.

ROS topic names and message types live in `gazebo_bridge.launch.py`. The installed Harmonic bridge has no conversion for Gazebo's scene, scene-deletion, world-statistics, marker, or serialized-state messages. Command-only Gazebo inputs and Gazebo services are outside this telemetry launch. Use MAVROS for flight-controller commands.

If running MAVROS separately with the telemetry bridge, disable MAVROS's own clock bridge.

```bash
ros2 launch robot_description sim_apm_udp.launch.py bridge_clock:=false
```

## Run the tether supervisor

In one sourced terminal, launch the tethered Quad X.

```bash
ros2 launch robot_description copter_sim.launch.py world:=quad_x_tether mode:=gui
```

In a second sourced terminal, start the supervisor.

```bash
ros2 run robot_description tethered_drone_node --ros-args -p use_sim_time:=true
```

Use `ros2 node info /winch_supervisor` from another sourced terminal to inspect the connections. The node starts in monitoring mode. Flight and winch outputs are explicit C++ calls in `WinchSupervisor`. The tension control law is a separate step.

- **Aircraft telemetry** comes from MAVROS `state`, `extended_state`, `estimator_status`, `local_position/odom`, `local_position/velocity_local`, and `global_position/rel_alt`. Odometry provides the local pose and body-frame twist. `velocity_local` provides world-frame velocity for climb and line-direction calculations.
- **Winch telemetry** comes from `/gz/tether/` and includes length, range, extension, payout rate, tension, azimuth, elevation, drum load torque, and anchor force. Length and payout rate are the drum observables the controller needs. Angles and force use Gazebo world axes. Use the latest received samples and their local receipt times from `tether_telemetry()` and `quad_telemetry()`. The scalar tether messages carry no source timestamp.
- **Flight commands** use `publish_local_position()` or `publish_local_velocity()`. Both send to `/mavros/setpoint_raw/local` and require a recent connected, armed `GUIDED` state. Supply metres or metres per second in the MAVROS local **ENU** frame. MAVROS converts to MAVLink NED. A controller must periodically resend velocity targets and should be the only source of flight setpoints while it is active. See [ArduCopter Guided commands](https://ardupilot.org/dev/docs/copter-commands-in-guided-mode.html).
- **Winch commands** use `publish_winch_length()` to send a target in metres to `/gz/tether/cmd_length`. Gazebo applies the configured travel limits.
- **Flight services** use `request_mode()`, `request_arm()`, `request_takeoff()`, and `request_land()`. Takeoff uses altitude in metres above home and requires armed `GUIDED`. These methods return an optional asynchronous request handle. An empty result means the request was not sent. Check the service response and confirm flight state through telemetry before advancing. Poll futures from a timer without blocking the executor.

The default endpoint prefixes are `/mavros` and `/gz/tether`. Override them with the `mavros_namespace` and `tether_namespace` parameters using absolute names without a trailing slash. Gazebo world and MAVROS local frames have independent origins. A controller combining their positions must align the frames first.

## Connect MAVROS to a physical flight controller

Run only the MAVROS launch, use wall time, and point `fcu_url` at the controller. Substitute the actual serial device and configured baud rate.

```bash
ros2 launch robot_description sim_apm_udp.launch.py \
  fcu_url:=serial:///dev/ttyACM0:115200 use_sim_time:=false bridge_clock:=false
```

The same launch accepts a MAVLink UDP endpoint. The default simulation listener is `udp://127.0.0.1:14560@`. Use `namespace`, `tgt_system`, and `tgt_component` for independently configured vehicles.
