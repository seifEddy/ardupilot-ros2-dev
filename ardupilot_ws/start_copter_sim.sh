# This script is going gto be used to start the simulation environment for the ArduPilot ROS2 development workspace. It sets up the necessary environment variables and sources the required scripts for proper functionality.
set -ex

# Kill all SITL binaries when exiting
trap "killall -9 arducopter" SIGINT SIGTERM EXIT

export GZ_VERSION=harmonic

SCRIPT_DIR="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)"

# First we check if we have built the ArduPilot firmware for SITL board. If not, we will build it.
if [ ! -d "${SCRIPT_DIR}/ardupilot/build" ]; then
    echo -e "\033[1;32mArduPilot firmware not found. Building it now...\033[0m" 
    cd "${SCRIPT_DIR}/ardupilot"
    git submodule update --init --recursive
    ./waf configure --board sitl
    ./waf copter
    cd "${SCRIPT_DIR}"
    echo -e "\033[1;32mArduPilot firmware built successfully.\033[0m"
fi

# Second, we check if the user has built the ArduPilot Gazebo plugin. If not, we will build it.
if [ ! -d "${SCRIPT_DIR}/gz_ws/src/ardupilot_gazebo/build" ]; then
    echo -e "\033[1;32mArduPilot Gazebo plugin not found. Building it now...\033[0m" 
    cd "${SCRIPT_DIR}/gz_ws/src/ardupilot_gazebo"
    mkdir -p build
    cd build
    cmake .. -DCMAKE_BUILD_TYPE=RelWithDebInfo
    make -j4 # Just use 4 threads for building, you can change this number based on your CPU cores
    cd "${SCRIPT_DIR}"
    echo -e "\033[1;32mArduPilot Gazebo plugin built successfully.\033[0m"
fi

export GZ_SIM_SYSTEM_PLUGIN_PATH=${SCRIPT_DIR}/gz_ws/src/ardupilot_gazebo/build:$GZ_SIM_SYSTEM_PLUGIN_PATH
export GZ_SIM_RESOURCE_PATH=${SCRIPT_DIR}/gz_ws/src/ardupilot_gazebo/models:${SCRIPT_DIR}/gz_ws/src/ardupilot_gazebo/worlds:$GZ_SIM_RESOURCE_PATH

#
COPTER=${SCRIPT_DIR}/ardupilot/build/sitl/bin/arducopter

${COPTER} -S --model JSON --uartA mcast: &

gz sim -v4 -r iris_runway.sdf
