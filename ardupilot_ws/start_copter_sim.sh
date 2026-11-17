#!/usr/bin/env bash

set -e

export GZ_VERSION=harmonic

SCRIPT_DIR="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)"
GAZEBO_DIR="${SCRIPT_DIR}/gz_ws/src/ardupilot_gazebo"

case "${1:-headless}" in
    gui) gazebo_args=() ;;
    headless) gazebo_args=(-s --headless-rendering) ;;
    *)
        printf 'Usage: %s [gui|headless] [quad_x|quad_x_tether|iris]\n' "$0" >&2
        exit 2
        ;;
esac

case "${2:-quad_x}" in
    quad_x) world=quad_x.sdf ;;
    quad_x_tether) world=quad_x_tether.sdf ;;
    iris) world=iris_runway.sdf ;;
    *)
        printf 'Usage: %s [gui|headless] [quad_x|quad_x_tether|iris]\n' "$0" >&2
        exit 2
        ;;
esac


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
if [ ! -f "${GAZEBO_DIR}/build/libArduPilotPlugin.so" ] ||
   { [ "${world}" = quad_x_tether.sdf ] && [ ! -f "${GAZEBO_DIR}/build/libTetherPlugin.so" ]; }; then
    echo -e "\033[1;32mArduPilot Gazebo plugin not found. Building it now...\033[0m" 
    cd "${GAZEBO_DIR}"
    mkdir -p build
    cd build
    cmake .. -DCMAKE_BUILD_TYPE=RelWithDebInfo
    make -j4 # Just use 4 threads for building, you can change this number based on your CPU cores
    cd "${SCRIPT_DIR}"
    echo -e "\033[1;32mArduPilot Gazebo plugin built successfully.\033[0m"
fi

export GZ_SIM_SYSTEM_PLUGIN_PATH="${GAZEBO_DIR}/build${GZ_SIM_SYSTEM_PLUGIN_PATH:+:${GZ_SIM_SYSTEM_PLUGIN_PATH}}"
export GZ_SIM_RESOURCE_PATH="${GAZEBO_DIR}/models:${GAZEBO_DIR}/worlds${GZ_SIM_RESOURCE_PATH:+:${GZ_SIM_RESOURCE_PATH}}"

COPTER="${SCRIPT_DIR}/ardupilot/build/sitl/bin/arducopter"
SITL_DIR="${SITL_DIR:-${SCRIPT_DIR}/copter_files}"
mkdir -p "${SITL_DIR}/scripts"

# Stop only the processes owned by this launch.
sitl_pid=
gazebo_pid=
cleanup() {
    local -a targets=()
    [[ -z "${gazebo_pid}" ]] || targets+=("-${gazebo_pid}")
    [[ -z "${sitl_pid}" ]] || targets+=("${sitl_pid}")
    if (( ${#targets[@]} )); then
        kill -TERM -- "${targets[@]}" 2>/dev/null || true
        for _ in {1..20}; do
            local running=false
            for target in "${targets[@]}"; do
                if kill -0 -- "${target}" 2>/dev/null; then running=true; fi
            done
            if ! "${running}"; then break; fi
            sleep 0.1
        done
        kill -KILL -- "${targets[@]}" 2>/dev/null || true
        wait 2>/dev/null || true
    fi
}
trap cleanup EXIT
trap 'exit 130' INT
trap 'exit 143' TERM

(
    cd "${SITL_DIR}"
    exec "${COPTER}" -S --model JSON \
      --home=-35.363262,149.165237,584,90 \
      --defaults "${GAZEBO_DIR}/config/gazebo-iris-gimbal.parm" \
      --uartA mcast: \
      --serial2=udpclient:127.0.0.1:14560
) &
sitl_pid=$!

setsid gz sim -v4 -r "${GAZEBO_DIR}/worlds/${world}" "${gazebo_args[@]}" &
gazebo_pid=$!
wait -n "${sitl_pid}" "${gazebo_pid}"
