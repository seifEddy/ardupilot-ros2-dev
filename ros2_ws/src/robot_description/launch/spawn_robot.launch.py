"""
spawn_robot.launch.py
─────────────────────
Spawns differential-drive robot into a **running** Gazebo Harmonic
(gz sim) world and brings up the ros2_control stack:

  1. robot_state_publisher  – publishes /robot_description (processed XACRO)
  2. ros_gz_sim create      – spawns the URDF entity in the running gz sim
  3. joint_state_broadcaster spawner   (after 4 s, waits for gz plugin)
  4. diff_drive_controller  spawner    (after 5 s)
  5. robot_controller_node  demo node  (after 7 s, waits for controllers)

Usage
─────
  ros2 launch robot_description spawn_robot.launch.py
  ros2 launch robot_description spawn_robot.launch.py x:=2.0 y:=1.0

Prerequisites
─────────────
  • gz sim with any world already running
  • ros_gz_bridge / ros_gz_sim present (ros-humble-ros-gz or ros-humble-ros-gz-sim)
  • gz_ros2_control installed (ros-humble-gz-ros2-control)
"""

from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    TimerAction,
    RegisterEventHandler,
    LogInfo,
)
from launch.event_handlers import OnProcessExit
from launch.substitutions import (
    LaunchConfiguration,
    Command,
    FindExecutable,
    PathJoinSubstitution,
)
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
from launch_ros.parameter_descriptions import ParameterValue

def create_controller_spawner(controller_name, inactive=False):
    arguments = [
        controller_name,
        "--controller-manager",
        "/controller_manager",
        "--controller-manager-timeout",
        "60",
    ]

    if inactive:
        arguments.append("--inactive")

    return Node(
        package="controller_manager",
        executable="spawner",
        name=f"spawn_{controller_name}",
        output="screen",
        arguments=arguments,
    )

def generate_launch_description():

    # ── launch arguments ──────────────────────────────────────────────────────
    declare_x  = DeclareLaunchArgument("x", default_value="0.0",
                                       description="Spawn X position (m)")
    declare_y  = DeclareLaunchArgument("y", default_value="0.0",
                                       description="Spawn Y position (m)")
    declare_z  = DeclareLaunchArgument("z", default_value="0.30",
                                       description="Spawn Z position (m)")
    declare_yaw = DeclareLaunchArgument("yaw", default_value="0.0",
                                        description="Spawn yaw angle (rad)")
    declare_robot_name = DeclareLaunchArgument(
        "robot_name", default_value="robot",
        description="Entity (robot) name in Gazebo")

    # ── XACRO → URDF string ───────────────────────────────────────────────────
    robot_description_content = ParameterValue(
        Command([
            PathJoinSubstitution([FindExecutable(name="xacro")]),
            " ",
            PathJoinSubstitution([
                FindPackageShare("robot_description"),
                "urdf", "robot.urdf.xacro",
            ]),
        ]),
        value_type=str,
    )

    # ── robot_state_publisher ─────────────────────────────────────────────────
    robot_state_publisher = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        output="screen",
        parameters=[{"robot_description": robot_description_content,
                     "use_sim_time": True}],
    )

    # ── spawn entity into the running gz sim world ────────────────────────────
    gz_spawn = Node(
        package="ros_gz_sim",
        executable="create",
        output="screen",
        arguments=[
            "-name",  LaunchConfiguration("robot_name"),
            "-topic", "robot_description",
            "-x",     LaunchConfiguration("x"),
            "-y",     LaunchConfiguration("y"),
            "-z",     LaunchConfiguration("z"),
            "-Y",     LaunchConfiguration("yaw"),
        ],
    )

    # ── ros2_control: load joint_state_broadcaster ────────────────────────────
    # Give Gazebo time to spawn the entity and start the gz_ros2_control plugin.
    load_jsb = create_controller_spawner("joint_state_broadcaster", inactive=False)

    # ── ros2_control: load diff_drive_controller ──────────────────────────────
    load_ddc = create_controller_spawner("diff_drive_controller", inactive=False)

    # ── demo C++ controller node ──────────────────────────────────────────────
    controller_node = TimerAction(
        period=7.0,
        actions=[
            Node(
                package="robot_description",
                executable="robot_controller_node",
                output="screen",
                parameters=[{"use_sim_time": True}],
            )
        ],
    )

    return LaunchDescription([
        declare_x,
        declare_y,
        declare_z,
        declare_yaw,
        declare_robot_name,
        robot_state_publisher,
        gz_spawn,
        load_jsb,
        load_ddc,
        controller_node,
    ])
