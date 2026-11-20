# ArduPilot SITL and Gazebo

`start_copter_sim.sh` starts ArduCopter SITL and Gazebo Harmonic. It builds the SITL firmware and the ArduPilot Gazebo plugin on first use when their build directories are missing. Initialize the repository submodules and install the ArduPilot build dependencies and [Gazebo Harmonic](https://gazebosim.org/docs/harmonic/install_ubuntu/) first. See the [ArduPilot Gazebo setup](https://ardupilot.org/dev/docs/sitl-with-gazebo.html) for plugin prerequisites.

From the repository root, run:

```bash
./ardupilot_ws/start_copter_sim.sh                # Quad X with headless rendering
./ardupilot_ws/start_copter_sim.sh gui            # Quad X with the Gazebo GUI
./ardupilot_ws/start_copter_sim.sh gui quad_x_tether # Quad X with the ground winch
./ardupilot_ws/start_copter_sim.sh gui quad_x_tether_car # Quad X with a Prius-mounted winch
./ardupilot_ws/start_copter_sim.sh gui iris       # Original Iris runway
```

`gz_ws/src/ardupilot_gazebo/worlds/quad_x.sdf` loads the Quad X wrapper model, which adds Iris-based motor dynamics, the front RGB gimbal and the ArduPilot JSON plugin to the COLLADA airframe. The rear depth camera points down. The launcher sets the Gazebo resource paths, and SITL and Gazebo exchange JSON on loopback port 9002. The simulator's MAVLink serial port sends to MAVROS on UDP port 14560. MAVProxy uses the separate multicast link.

```bash
bash ./ardupilot_ws/start_mavproxy.sh
```

For simulation with MAVROS and the ROS 2 sensor bridge, use [the combined ROS 2 launch](../ros2_ws/README.md#run-the-combined-simulation). That launch starts this script and owns its lifetime. The script also works directly for a Gazebo and MAVProxy workflow. Simulator parameters, logs, and Lua scripts use `copter_files/` by default. Set `SITL_DIR` to an alternate absolute directory to run with separate simulator state.

## Tethered Quad X

The tether world adds an anchored station, driven drum and compliant line to the Quad X test field. The initial payout is 3.0 m. Above that length, automatic take-up applies a gentle winding torque to recover slack. The aircraft can pull cable out when tension overcomes the take-up torque and brake resistance. Explicit length commands temporarily take control of the reel.

The current world uses `takeup_tension=0.25` N, `brake_damping=0.00098` N·m·s/rad, and 2.0 m/s retraction and payout speed limits. Cable stiffness is 10.0 N/m, axial damping is 1.0 N·s/m, and the cable force saturates at 15.0 N. Speed limits cap motion, while torque, damping, cable load, and drum inertia determine the actual response. See the [tether model and measurements](gz_ws/src/ardupilot_gazebo/docs/quad_x_tether.md) for the joint layout, force law, tuning calculations, and winch commands.

The [ROS 2 tether supervisor](../ros2_ws/README.md#run-the-tether-supervisor) adds station-relative XYZ proportional control through MAVROS velocity targets. Its target altitude is adjustable at runtime with `target_altitude_z` and its X and Y targets are zero.

### Rebuild the tether plugin

After editing the plugin C++ source, build the shared library from the repository checkout.

```bash
cd ~/ardupilot-ros2-dev/ardupilot_ws/gz_ws/src/ardupilot_gazebo
GZ_VERSION=harmonic CCACHE_DISABLE=1 cmake -S . -B build -DCMAKE_BUILD_TYPE=RelWithDebInfo
CCACHE_DISABLE=1 cmake --build build --target TetherPlugin -j4
```

Restart Gazebo to load the rebuilt library. Changes to the world SDF also take effect after restarting Gazebo. The launcher builds missing plugin libraries on startup. Rebuild explicitly after source changes to refresh an existing library.

## Mobile Prius tether station

`quad_x_tether_car` uses a simplified local copy of the Prius from the Open Robotics Sonoma example. Its front wheels steer, rear wheels drive, and the winch base is fixed to the roof. The existing drum and cable plugin handle moving-anchor forces. ROS keyboard commands and car odometry are available through the mobile bridge profile.

`station_mount_joint` fixes the station base to the chassis, replacing the ground anchor in this mobile model. The independent `drum_joint` retains its payout and retraction dynamics. The station base is 1.5 m above the car origin, and its fairlead attachment is at `(0, 0, 1.82)` m in the car frame. The plugin calculates cable extension rate using velocities at both moving attachment points.

The ROS keyboard node publishes `/car/cmd_vel` and the bridge exposes car odometry, 3D ground truth, and joint states. `car_teleop.py` uses W/S for forward/reverse, A/D for forward turns, Z/C for reverse turns, and Space to stop. It commands zero after its key timeout and on normal exit.

See the [mobile tether guide](gz_ws/src/ardupilot_gazebo/docs/quad_x_tether_car.md) for model sources, mounting geometry, controls, and topics. The [ROS demo workflow](../ros2_ws/README.md#run-the-15-m-to-6-m-altitude-demo) starts the complete stack with keyboard control and PlotJuggler, then changes `target_altitude_z` from its 15 m default to 6 m. The [mobile demo GIF](../README.md#mobile-car-tracking-and-altitude-step) shows the Gazebo view and relative-position plots.

## Circle flight demo

`copter_files/scripts/circle_demo.lua` is an opt-in SITL demo. Enable Lua with `SCR_ENABLE=1` in MAVProxy and restart SITL. Once the vehicle is disarmed and EKF and home are ready, set the demo parameters in the MAVProxy console.

```text
param set CDEM_MODE 1
param set CDEM_ALT 10
param set CDEM_RADIUS 10
param set CDEM_SPEED 2
param set CDEM_LAPS 1
param set CDEM_DO_RTL 0
param set CDEM_START 1
```

Mode `1` flies a continuous circle. Mode `2` flies polygon vertices and uses `CDEM_NPTS` for their count. The demo arms, takes off in GUIDED mode, flies the path, and requests LAND at completion. Set `CDEM_DO_RTL=1` to request RTL instead. Set `CDEM_START=0` to cancel and land while the demo is still in control. An operator mode change releases script control. The start flag resets on script startup.
