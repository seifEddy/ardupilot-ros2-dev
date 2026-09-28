"""Launch stock MAVROS APM configuration and request telemetry over UDP.

Installed by the robot_description ament_cmake package. Run:
  ros2 launch robot_description apm_udp.launch.py
"""

from pathlib import Path

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterFile, ParameterValue


def generate_launch_description():
    mavros_share = Path(get_package_share_directory("mavros"))
    package_share = Path(get_package_share_directory("robot_description"))
    defaults = {
        "fcu_url": ("udp://127.0.0.1:14560@", "MAVROS UDP listening endpoint"),
        "namespace": ("mavros", "MAVROS namespace"),
        "tgt_system": ("1", "Vehicle MAVLink system ID"),
        "tgt_component": ("1", "Vehicle MAVLink autopilot component ID"),
        "stream_id": ("0", "MAVLink stream group: 0=all, 6=position, 10=extra1"),
        "stream_rate": ("10", "Requested telemetry rate in Hz, integer 1..100"),
        "use_sim_time": ("true", "Use Gazebo /clock in MAVROS and the requester"),
        "bridge_clock": ("true", "Start a clock bridge; disable if one already exists"),
        "gz_clock_topic": ("/clock", "Gazebo clock topic, possibly /world/<name>/clock"),
    }
    arguments = [
        DeclareLaunchArgument(name, default_value=value, description=description)
        for name, (value, description) in defaults.items()
    ]
    sim_time = ParameterValue(LaunchConfiguration("use_sim_time"), value_type=bool)
    clock_bridge = Node(
        package="ros_gz_bridge",
        executable="parameter_bridge",
        name="gazebo_clock_bridge",
        arguments=[[
            LaunchConfiguration("gz_clock_topic"),
            "@rosgraph_msgs/msg/Clock[gz.msgs.Clock",
        ]],
        remappings=[(LaunchConfiguration("gz_clock_topic"), "/clock")],
        parameters=[{"use_sim_time": False}],
        condition=IfCondition(LaunchConfiguration("bridge_clock")),
        output="screen",
    )
    # Same APM configuration as stock apm.launch. Do not set a global node
    # name: mavros_node creates several internal nodes and plugin nodes.
    mavros = Node(
        package="mavros",
        executable="mavros_node",
        namespace=LaunchConfiguration("namespace"),
        parameters=[
            str(mavros_share / "launch" / "apm_pluginlists.yaml"),
            str(mavros_share / "launch" / "apm_config.yaml"),
            {
                "fcu_url": ParameterValue(LaunchConfiguration("fcu_url"), value_type=str),
                "gcs_url": "",
                "tgt_system": ParameterValue(LaunchConfiguration("tgt_system"), value_type=int),
                "tgt_component": ParameterValue(LaunchConfiguration("tgt_component"), value_type=int),
                "fcu_protocol": "v2.0",
            },
            ParameterFile(str(package_share / "config" / "mavros_clock.yaml"), allow_substs=True),
        ],
        output="screen",
    )
    configure_streams = Node(
        package="robot_description",
        executable="request_streams.py",
        namespace=LaunchConfiguration("namespace"),
        name="stream_setup",
        parameters=[{
            "use_sim_time": sim_time,
            "stream_id": ParameterValue(LaunchConfiguration("stream_id"), value_type=int),
            "stream_rate": ParameterValue(LaunchConfiguration("stream_rate"), value_type=int),
        }],
        output="screen",
    )

    clock_bridge = Node(
        package="ros_gz_bridge",
        executable="parameter_bridge",
        name="clock_bridge",
        arguments=[
            "/clock@rosgraph_msgs/msg/Clock[gz.msgs.Clock",
        ],
        output="screen",
    )
    return LaunchDescription([*arguments, mavros, configure_streams, clock_bridge])
