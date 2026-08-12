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
