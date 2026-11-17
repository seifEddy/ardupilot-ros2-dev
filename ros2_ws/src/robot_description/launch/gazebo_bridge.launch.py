"""Bridge Gazebo Harmonic telemetry for the Quad X test field or Iris runway."""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def configure_bridge(context):
    quad_x = LaunchConfiguration("world").perform(context) == "quad_x"
    world = "/world/quad_x_test_field" if quad_x else "/world/iris_runway"
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

    return [Node(
        package="ros_gz_bridge",
        executable="parameter_bridge",
        name="gazebo_telemetry_bridge",
        arguments=[f"{topic}@{ros_type}[gz.msgs.{gz_type}" for topic, _, ros_type, gz_type in mappings],
        remappings=[(topic, ros_topic) for topic, ros_topic, _, _ in mappings],
        output="screen",
    )]


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument(
            "world", default_value="quad_x", choices=["quad_x", "iris"],
            description="The world selected in start_copter_sim.sh",
        ),
        OpaqueFunction(function=configure_bridge),
    ])
