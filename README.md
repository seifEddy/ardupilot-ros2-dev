# ArduPilot ROS 2 development workspace

This repository brings ArduCopter SITL, Gazebo Harmonic, and ROS 2 Humble together for development. The Quad X test field is the default simulation. MAVROS provides the MAVLink connection used by both SITL and a physical flight controller. Gazebo sensor data is available separately through `ros_gz_bridge`.

Clone the repository with its submodules.

```bash
git clone --recurse-submodules https://github.com/seifEddy/ardupilot-ros2-dev.git
cd ardupilot-ros2-dev
```

- [ArduPilot workspace](ardupilot_ws/README.md) covers standalone simulation, the Gazebo models, MAVProxy, and the circle-flight demo.
- [ROS 2 workspace](ros2_ws/README.md) covers dependencies, the combined launch, Gazebo topics, and connecting MAVROS to a flight controller.

After building and sourcing the ROS 2 workspace, start the full simulation stack with:

```bash
ros2 launch robot_description copter_sim.launch.py mode:=gui
```
