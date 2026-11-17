# ArduPilot SITL and Gazebo

`start_copter_sim.sh` starts ArduCopter SITL and Gazebo Harmonic. It builds the SITL firmware and the ArduPilot Gazebo plugin on first use when their build directories are missing. Initialize the repository submodules and install the ArduPilot build dependencies and [Gazebo Harmonic](https://gazebosim.org/docs/harmonic/install_ubuntu/) first. See the [ArduPilot Gazebo setup](https://ardupilot.org/dev/docs/sitl-with-gazebo.html) for plugin prerequisites.

From the repository root, run:

```bash
./ardupilot_ws/start_copter_sim.sh                # Quad X with headless rendering
./ardupilot_ws/start_copter_sim.sh gui            # Quad X with the Gazebo GUI
./ardupilot_ws/start_copter_sim.sh gui iris       # Original Iris runway
```

`gz_ws/src/ardupilot_gazebo/worlds/quad_x.sdf` loads the Quad X wrapper model, which adds Iris-based motor dynamics, the front RGB gimbal and the ArduPilot JSON plugin to the COLLADA airframe. The rear depth camera points down. The launcher sets the Gazebo resource paths, and SITL and Gazebo exchange JSON on loopback port 9002. The simulator's MAVLink serial port sends to MAVROS on UDP port 14560. MAVProxy uses the separate multicast link.

```bash
bash ./ardupilot_ws/start_mavproxy.sh
```

For simulation with MAVROS and the ROS 2 sensor bridge, use [the combined ROS 2 launch](../ros2_ws/README.md#run-the-combined-simulation). That launch starts this script and owns its lifetime. The script also works directly for a Gazebo and MAVProxy workflow. Simulator parameters, logs, and Lua scripts use `copter_files/` by default. Set `SITL_DIR` to an alternate absolute directory to run with separate simulator state.

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
