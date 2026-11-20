"""Bridge Gazebo Harmonic telemetry for the Quad X test field or Iris runway."""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def configure_bridge(context):
    selected_world = LaunchConfiguration("world").perform(context)
    tethered = selected_world in ("quad_x_tether", "quad_x_tether_car")
    mobile = selected_world == "quad_x_tether_car"
    quad_x = selected_world != "iris"
    world = "/world/" + {
        "quad_x": "quad_x_test_field",
        "quad_x_tether": "quad_x_tether_field",
        "quad_x_tether_car": "quad_x_tether_car_field",
        "iris": "iris_runway",
    }[selected_world]
    vehicle = f"{world}/model/{'quad_x_with_gimbal' if quad_x else 'iris_with_gimbal'}"
    airframe = f"{vehicle}/model/{'quad_x' if quad_x else 'iris_with_standoffs'}"
    camera = f"{vehicle}/model/gimbal/link/pitch_link/sensor/camera"

    # Keep all simulation telemetry under /gz, except the standard ROS clock.
    mappings = [
        (f"{world}/clock", "/clock", "rosgraph_msgs/msg/Clock", "Clock"),
        (f"{world}/pose/info", "/gz/world/poses", "geometry_msgs/msg/PoseArray", "Pose_V"),
        (f"{world}/dynamic_pose/info", "/gz/world/dynamic_poses", "geometry_msgs/msg/PoseArray", "Pose_V"),
        ("/gazebo/resource_paths", "/gz/world/resource_paths", "ros_gz_interfaces/msg/StringVec", "StringMsg_V"),
        (f"{vehicle}/joint_state", "/gz/joint_states", "sensor_msgs/msg/JointState", "Model"),
        (f"{airframe}/link/imu_link/sensor/imu_sensor/imu", "/gz/imu", "sensor_msgs/msg/Imu", "IMU"),
        (f"{camera}/image", "/gz/camera/front/image", "sensor_msgs/msg/Image", "Image"),
        (f"{camera}/camera_info", "/gz/camera/front/camera_info", "sensor_msgs/msg/CameraInfo", "CameraInfo"),
    ]
    for axis in ("pitch", "roll", "yaw"):
        mappings.append((f"/gimbal/cmd_{axis}", f"/gz/gimbal/cmd_{axis}", "std_msgs/msg/Float64", "Double"))

    if quad_x:
        sensors = f"{airframe}/link/base_link/sensor"
        mappings.extend([
            (f"{sensors}/air_pressure/air_pressure", "/gz/air_pressure", "sensor_msgs/msg/FluidPressure", "FluidPressure"),
            (f"{sensors}/magnetometer/magnetometer", "/gz/magnetometer", "sensor_msgs/msg/MagneticField", "Magnetometer"),
            (f"{sensors}/navsat/navsat", "/gz/navsat", "sensor_msgs/msg/NavSatFix", "NavSat"),
            ("/quad_x/rear_depth", "/gz/camera/rear_depth/image", "sensor_msgs/msg/Image", "Image"),
            ("/quad_x/rear_depth/camera_info", "/gz/camera/rear_depth/camera_info", "sensor_msgs/msg/CameraInfo", "CameraInfo"),
            ("/quad_x/rear_depth/points", "/gz/camera/rear_depth/points", "sensor_msgs/msg/PointCloud2", "PointCloudPacked"),
        ])
    else:
        mappings.append((
            f"{world}/model/axes/link/link/sensor/navsat_sensor/navsat",
            "/gz/world/reference_navsat", "sensor_msgs/msg/NavSatFix", "NavSat",
        ))

    if tethered:
        for name in (
            "length", "range", "extension", "payout_rate", "tension",
            "azimuth", "elevation", "drum_load_torque",
        ):
            mappings.append((f"/tether/{name}", f"/gz/tether/{name}", "std_msgs/msg/Float64", "Double"))
        mappings.append(("/tether/anchor_force", "/gz/tether/anchor_force", "geometry_msgs/msg/Vector3", "Vector3d"))

    if mobile:
        mappings.extend([
            ("/model/tether_car/odometry", "/car/odom", "nav_msgs/msg/Odometry", "Odometry"),
            ("/model/tether_car/ground_truth", "/car/ground_truth", "nav_msgs/msg/Odometry", "Odometry"),
            ("/model/tether_car/joint_state", "/car/joint_states", "sensor_msgs/msg/JointState", "Model"),
        ])

    arguments = [f"{topic}@{ros_type}[gz.msgs.{gz_type}" for topic, _, ros_type, gz_type in mappings]
    remappings = [(topic, ros_topic) for topic, ros_topic, _, _ in mappings]
    if tethered:
        arguments.append("/tether/cmd_length@std_msgs/msg/Float64]gz.msgs.Double")
        remappings.append(("/tether/cmd_length", "/gz/tether/cmd_length"))

    if mobile:
        arguments.append("/model/tether_car/cmd_vel@geometry_msgs/msg/Twist]gz.msgs.Twist")
        remappings.append(("/model/tether_car/cmd_vel", "/car/cmd_vel"))

    return [Node(
        package="ros_gz_bridge",
        executable="parameter_bridge",
        name="gazebo_telemetry_bridge",
        arguments=arguments,
        remappings=remappings,
        output="screen",
    )]


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument(
            "world", default_value="quad_x", choices=["quad_x", "quad_x_tether", "quad_x_tether_car", "iris"],
            description="The world selected in start_copter_sim.sh",
        ),
        OpaqueFunction(function=configure_bridge),
    ])
