"""Start ArduPilot SITL, Gazebo Harmonic, MAVROS, and the Gazebo bridge."""

from pathlib import Path

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    ExecuteProcess,
    IncludeLaunchDescription,
    OpaqueFunction,
    Shutdown,
)
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration


def default_script(package_share):
    relative_script = Path("ardupilot_ws/start_copter_sim.sh")
    for directory in (Path.cwd(), *package_share.resolve().parents):
        candidate = directory / relative_script
        if candidate.is_file():
            return str(candidate)
    return str(Path.cwd() / relative_script)


def start_stack(context, package_share):
    script = Path(LaunchConfiguration("sitl_script").perform(context)).expanduser().resolve()
    if not script.is_file():
        raise RuntimeError(f"SITL script not found at {script}. Set sitl_script to its absolute path.")

    def include(filename, arguments):
        return IncludeLaunchDescription(
            PythonLaunchDescriptionSource(str(package_share / "launch" / filename)),
            launch_arguments=arguments.items(),
        )

    return [
        ExecuteProcess(
            cmd=["bash", str(script), LaunchConfiguration("mode"), LaunchConfiguration("world")],
            name="copter_sim",
            output="screen",
            on_exit=Shutdown(reason="The SITL or Gazebo process exited"),
        ),
        include("gazebo_bridge.launch.py", {"world": LaunchConfiguration("world")}),
        include("sim_apm_udp.launch.py", {
            "fcu_url": LaunchConfiguration("fcu_url"),
            "use_sim_time": "true",
            "bridge_clock": "false",
        }),
    ]


def generate_launch_description():
    package_share = Path(get_package_share_directory("robot_description"))
    return LaunchDescription([
        DeclareLaunchArgument("mode", default_value="headless", choices=["headless", "gui"]),
        DeclareLaunchArgument("world", default_value="quad_x", choices=["quad_x", "quad_x_tether", "quad_x_tether_car", "iris"]),
        DeclareLaunchArgument("fcu_url", default_value="udp://127.0.0.1:14560@"),
        DeclareLaunchArgument(
            "sitl_script", default_value=default_script(package_share),
            description="Path to start_copter_sim.sh; override if the ROS install is outside the checkout",
        ),
        OpaqueFunction(function=start_stack, args=[package_share]),
    ])
