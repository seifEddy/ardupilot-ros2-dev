# ardupilot-ros2-dev
This repository holds the basic Ardupilot and ROS2 basic project to start with.

When you clone this repo, rememeber to recursively sync and init the submodules:

```bash
git clone --recurse-submodules https://github.com/seifEddy/ardupilot-ros2-dev.git
```

## Start Copter simulation (iris_runway)

Assuming that you cloned the repository including all the submodules, all you have to do is:

```bash
chmod +x ardupilot_ws/start_copter_sim.sh
./ardupilot_ws/start_copter_sim.sh
```

This script will: 

- Check if ardupilot firmware was built for `sitl`, if not it will build it.
- Check if ardupilot Gazebo plugin was built, if not it will build it.
- Launch copter simulation, and it is ready to interact with using `mavproxy.py`

## ROS2

First, you need to have `ros2` humble installed here -> [ros2 humble installation](https://docs.ros.org/en/humble/Installation/Ubuntu-Install-Debs.html)

When you have installed `ros2` humble you can build the packages on the `ros2_ws`:

```bash
# From inside the cloned repo
cd ros2_ws
rosdep install --from-paths src --ignore-src -y
colcon build
source install/setup.bash  # Or add it permanently to you ~/.bashrc
```

After a successful build, you now can launch the `ros2` robot spawner to have it up and running in your already running `gz` instance.

```bash
ros2 launch robot_description spawn_robot.launch.py
```
