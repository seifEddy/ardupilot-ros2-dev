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

The default is `mode:=headless world:=quad_x`. Set `world:=quad_x_tether` for the fixed tether station, `world:=quad_x_tether_car` for the mobile Prius station, or `world:=iris` for the original Iris runway. If the ROS install is outside the checkout, pass `sitl_script:=/absolute/path/to/ardupilot_ws/start_copter_sim.sh`. Run a single stack per SITL endpoint.

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
ros2 run robot_description tethered_drone_node --ros-args \
  -p use_sim_time:=true -p target_altitude_z:=15.0
```

Use `ros2 node info /winch_supervisor` from another sourced terminal to inspect the connections. Starting this node activates the station-relative position-control demo. Its 100 ms wall timer requests GUIDED mode and arming, then requests takeoff before publishing XYZ velocity commands. Run this demo with the tethered SITL stack and give it ownership of the flight setpoints. The Gazebo plugin handles automatic reel take-up and payout. Tension-aware aircraft control is a future step.

- **Aircraft telemetry** comes from MAVROS `state`, `extended_state`, `estimator_status`, `local_position/odom`, `local_position/velocity_local`, and `global_position/rel_alt`. Odometry provides the local pose and body-frame twist. `velocity_local` provides world-frame velocity for climb and line-direction calculations.
- **Winch telemetry** comes from `/gz/tether/` and includes length, range, extension, payout rate, tension, azimuth, elevation, drum load torque, and anchor force. Length and payout rate are the drum observables the controller needs. Angles and force use Gazebo world axes. Use the latest received samples and their local receipt times from `tether_telemetry()` and `quad_telemetry()`. The scalar tether messages carry no source timestamp.
- **Flight commands** use `publish_local_position()` or `publish_local_velocity()`. Both send to `/mavros/setpoint_raw/local` and require a recent connected, armed `GUIDED` state. Supply metres or metres per second in the MAVROS local **ENU** frame. MAVROS converts to MAVLink NED. A controller must periodically resend velocity targets and should be the only source of flight setpoints while it is active. See [ArduCopter Guided commands](https://ardupilot.org/dev/docs/copter-commands-in-guided-mode.html).
- **Winch commands** use `publish_winch_length()` to send a target in metres to `/gz/tether/cmd_length`. Gazebo applies the configured travel limits.
- **Flight services** use `request_mode()`, `request_arm()`, `request_takeoff()`, and `request_land()`. Takeoff uses altitude in metres above home and requires armed `GUIDED`. These methods return an optional asynchronous request handle. An empty result means the request was not sent. Check the service response and confirm flight state through telemetry before advancing. Poll futures from a timer without blocking the executor.

The default endpoint prefixes are `/mavros` and `/gz/tether`. Override them with the `mavros_namespace` and `tether_namespace` parameters using absolute names without a trailing slash. Gazebo world and MAVROS local frames have independent origins. A controller combining their positions must align the frames first.

### Position controller and target altitude

The controller reconstructs the aircraft attachment position relative to the station fairlead from endpoint `range`, `azimuth`, and `elevation`. Angles are in radians and the vector uses Gazebo world ENU axes.

```text
x = range × cos(elevation) × cos(azimuth)
y = range × cos(elevation) × sin(azimuth)
z = range × sin(elevation)

vx = 0.80 × (0.0 − x)
vy = 0.80 × (0.0 − y)
vz = 1.20 × (target_altitude_z − z)
```

The target is `(0.0, 0.0, target_altitude_z)` metres relative to the fairlead. The X and Y targets and the proportional gains are hardcoded. ArduPilot's velocity and attitude controllers track the velocity commands in GUIDED mode.

`target_altitude_z` is a double ROS parameter with a default of `15.0` m. The timer reads it on every callback, so it can be changed while the node runs.

```bash
ros2 param get /winch_supervisor target_altitude_z
ros2 param set /winch_supervisor target_altitude_z 6.0
```

Use a finite value greater than 0.5 m. The callback skips control when this condition is unmet. The parameter service can still accept a value that the callback subsequently skips.

The node publishes the reconstructed fairlead-to-aircraft attachment vector on `/gz/tether/relative_position` as `geometry_msgs/msg/Vector3`. This message has no header or timestamp. Its components use world ENU axes and its origin follows the fairlead. For comparison with MAVROS aircraft body position, align the world and local frames and account for the fairlead and aircraft attachment offsets.

```bash
ros2 topic echo /gz/tether/relative_position
ros2 topic echo /mavros/local_position/odom
ros2 topic echo /gz/tether/tension
```

### Current takeoff sequencing

The demo requests a hardcoded takeoff altitude of 10 m above home whenever paid-out cable length is below `target_altitude_z / 3.0`. Above that threshold it runs the proportional controller. The takeoff altitude and the controller's target use different reference points. Increasing the target sufficiently can enter the takeoff branch again during flight. Mode, arming, and takeoff requests are retried by the timer according to telemetry, and the demo does not track their asynchronous responses. The length-based takeoff gate and service sequencing are part of the current prototype.

## MAVROS package and namespace troubleshooting

Keep `mavros`, `mavros-extras`, `mavros-msgs`, and `libmavconn` on compatible package versions after installing or restoring ROS packages. A mixed installation can make `mavros_node` exit with a missing message-type-support symbol. Update the related packages together and rebuild `robot_description` against the installed messages.

```bash
sudo apt install ros-humble-mavros ros-humble-mavros-extras \
  ros-humble-mavros-msgs ros-humble-libmavconn ros-humble-mavlink
```

The launch explicitly places MAVROS under `<namespace>/mavros`. With the default empty prefix, state is `/mavros/state` and command services include `/mavros/set_mode` and `/mavros/cmd/arming`. With MAVROS 2.17, giving the process an empty launch namespace can place plugin interfaces at `/state`, `/set_mode`, and `/cmd/arming`. The supervisor and stream requester expect the `/mavros` interfaces.

```bash
ros2 topic info /mavros/state
ros2 topic echo /mavros/state --once
ros2 service list | grep -E 'mavros/(set_mode|cmd/arming)'
```

A state-topic publisher confirms that the ROS interface is present. Its `connected` field confirms the MAVLink heartbeat connection. The supervisor also requires tether length and anchor-force messages before attempting mode or arming requests. Arming can still be rejected while the estimator and sensors initialize. Read `/mavros/statustext/recv` for the FCU reason. During the isolated startup check, `Arm: Accels inconsistent` cleared after settling and a subsequent request succeeded.

## Mobile winch car

The `quad_x_tether_car` world adds a simplified Prius with a roof-mounted winch and the tethered Quad X. The car retains the source body and wheel visuals, with a single body collision, rigid rear-wheel mounts, and front steering. The station base is attached to the car chassis by `station_mount_joint`, a fixed joint. The drum remains free to rotate about `drum_joint`. The fairlead is at `(0, 0, 1.82)` m in the car frame, and the tether plugin evaluates the moving anchor's pose and velocity every physics step.

The bridge accepts `geometry_msgs/msg/Twist` on `/car/cmd_vel` and publishes `nav_msgs/msg/Odometry` on `/car/odom` and `/car/ground_truth`, plus `sensor_msgs/msg/JointState` on `/car/joint_states`. Odometry uses a local `car/odom` frame. Ground truth uses `world` and includes the car spawn offset. Existing drone and tether topics are available in the same profile. To bridge a separately started mobile world, use `gazebo_bridge.launch.py world:=quad_x_tether_car`.

### Keyboard teleoperation

Run the installed ROS node from a sourced interactive terminal.

```bash
ros2 run robot_description car_teleop.py
```

Use W/S for forward/reverse, A/D for forward turns, Z/C for reverse turns, Space to stop, and Q or Ctrl+C to exit. The node sends commands at approximately 20 Hz, restores terminal settings on exit, and sends zero commands after 0.6 s without a key. Terminal key repeat determines how a held key is received. Startup parameters are `speed` (0.5 m/s), `turn_rate` (0.2 rad/s), and `key_timeout` (0.6 s).

```bash
ros2 run robot_description car_teleop.py --ros-args \
  -p speed:=0.25 -p turn_rate:=0.05 -p key_timeout:=0.6
```

Ackermann steering requires translational motion to turn. The car plugin caps speed at ±2 m/s and acceleration at ±1 m/s², so a zero command causes deceleration according to that limit. The key timeout runs inside the keyboard node. The car plugin retains its last command if that node terminates without sending a stop.

If your ROS CLI lacks the `run` extension, install `ros-humble-ros2run` or execute the ROS node directly after sourcing the workspace.

```bash
python3 ~/ardupilot-ros2-dev/ros2_ws/src/robot_description/scripts/car_teleop.py
```

### Run the 15 m to 6 m altitude demo

Build the package and source ROS plus the overlay in each terminal. Install PlotJuggler's ROS plugins if needed to visualize the states.

```bash
source /opt/ros/humble/setup.bash
cd ~/ardupilot-ros2-dev/ros2_ws
colcon build --packages-select robot_description
source install/setup.bash
```

In the first terminal, start Gazebo, SITL, MAVROS, and the bridge.

```bash
ros2 launch robot_description copter_sim.launch.py world:=quad_x_tether_car mode:=gui
```

In the second terminal, start the aircraft controller. Starting it activates its GUIDED, arming, and takeoff requests. Omitting `target_altitude_z` uses the default 15.0 m target above the moving fairlead. X and Y targets are zero.

```bash
ros2 run robot_description tethered_drone_node --ros-args -p use_sim_time:=true
```

In the third terminal, start `car_teleop.py` as shown above.

After the aircraft approaches 15 m relative altitude, change the live target from another sourced terminal.

```bash
ros2 param set /winch_supervisor target_altitude_z 6.0
ros2 param get /winch_supervisor target_altitude_z
```
The target is a ROS parameter. The current node publishes measured position. Drive the car from the keyboard terminal to observe the relative X/Y tracking response.

The [mobile demo GIF](../README.md#mobile-car-tracking-and-altitude-step) shows this workflow. The [mobile tether guide](../ardupilot_ws/gz_ws/src/ardupilot_gazebo/docs/quad_x_tether_car.md) describes the copied model, roof joint, topics, and moving-anchor dynamics.

### Current controller assumptions (Needs some LOVE)

The proportional gains are hardcoded at `Kp_xy=0.8` and `Kp_z=1.2` s⁻¹. A moving car can produce tracking lag because the controller uses relative-position feedback without car-velocity feed-forward. The controller currently has no explicit velocity-command saturation. Its target altitude is relative to the fairlead, and its takeoff service requests 10 m above home through the length-based gate described above. Tether sample availability and finite range/angle checks are implemented, while tether sample-age rejection and service-response sequencing are further development steps.

## Connect MAVROS to a physical flight controller

Run only the MAVROS launch, use wall time, and point `fcu_url` at the controller. Substitute the actual serial device and configured baud rate.

```bash
ros2 launch robot_description sim_apm_udp.launch.py \
  fcu_url:=serial:///dev/ttyACM0:115200 use_sim_time:=false bridge_clock:=false
```

The same launch accepts a MAVLink UDP endpoint. The default simulation listener is `udp://127.0.0.1:14560@`. Use `namespace`, `tgt_system`, and `tgt_component` for independently configured vehicles.
