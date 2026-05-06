import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import ComposableNodeContainer, Node
from launch_ros.descriptions import ComposableNode
from launch_ros.parameter_descriptions import ParameterValue


def compressed_republish(camera_name, image_topic, suffix=""):
    return Node(
        package="image_transport",
        executable="republish",
        name=f"{camera_name}{suffix}_compressed_republish",
        output="screen",
        arguments=["raw", "compressed"],
        remappings=[
            ("in", image_topic),
            ("in/image_raw", image_topic),
            ("out", image_topic),
            ("out/compressed", [image_topic, "/compressed"]),
            ("out/image_raw", image_topic),
            ("out/image_raw/compressed", [image_topic, "/compressed"]),
        ],
    )


def crop_decimate(camera_name, image_topic, camera_info_topic, decimated_topic, decimated_info_topic):
    return ComposableNode(
        package="image_proc",
        plugin="image_proc::CropDecimateNode",
        name=f"{camera_name}_crop_decimate",
        parameters=[
            {
                "decimation_x": ParameterValue(
                    LaunchConfiguration(f"{camera_name}_decimation_x"),
                    value_type=int,
                ),
                "decimation_y": ParameterValue(
                    LaunchConfiguration(f"{camera_name}_decimation_y"),
                    value_type=int,
                ),
                "offset_x": ParameterValue(
                    LaunchConfiguration(f"{camera_name}_offset_x"),
                    value_type=int,
                ),
                "offset_y": ParameterValue(
                    LaunchConfiguration(f"{camera_name}_offset_y"),
                    value_type=int,
                ),
                "width": ParameterValue(
                    LaunchConfiguration(f"{camera_name}_width"),
                    value_type=int,
                ),
                "height": ParameterValue(
                    LaunchConfiguration(f"{camera_name}_height"),
                    value_type=int,
                ),
            }
        ],
        remappings=[
            ("in/image", image_topic),
            ("in/image_raw", image_topic),
            ("in/camera_info", camera_info_topic),
            ("out/image", decimated_topic),
            ("out/image_raw", decimated_topic),
            ("out/image_raw/compressed", [decimated_topic, "/compressed"]),
            ("out/image_raw/compressedDepth", [decimated_topic, "/compressedDepth"]),
            ("out/image_raw/theora", [decimated_topic, "/theora"]),
            ("out/camera_info", decimated_info_topic),
        ],
    )


def generate_launch_description():
    camera_share = get_package_share_directory("sura_cameras")
    aruco_tracker_config = os.path.join(camera_share, "config", "aruco_tracker_down.yaml")

    down_image = LaunchConfiguration("down_camera_image_topic")
    down_info = LaunchConfiguration("down_camera_camera_info_topic")
    down_decimated = LaunchConfiguration("down_camera_decimated_topic")
    down_decimated_info = LaunchConfiguration("down_camera_decimated_camera_info_topic")
    front_image = LaunchConfiguration("front_camera_image_topic")
    front_info = LaunchConfiguration("front_camera_camera_info_topic")
    front_decimated = LaunchConfiguration("front_camera_decimated_topic")
    front_decimated_info = LaunchConfiguration("front_camera_decimated_camera_info_topic")

    return LaunchDescription(
        [
            DeclareLaunchArgument("launch_front_camera", default_value="true"),
            DeclareLaunchArgument("launch_down_aruco", default_value="true"),
            DeclareLaunchArgument(
                "down_camera_image_topic",
                default_value="/cirtesub/down_camera/camera/image_color",
            ),
            DeclareLaunchArgument(
                "down_camera_camera_info_topic",
                default_value="/cirtesub/down_camera/camera/camera_info",
            ),
            DeclareLaunchArgument(
                "down_camera_decimated_topic",
                default_value="/cirtesub/down_camera/decimated/image_raw",
            ),
            DeclareLaunchArgument(
                "down_camera_decimated_camera_info_topic",
                default_value="/cirtesub/down_camera/decimated/camera_info",
            ),
            DeclareLaunchArgument("down_camera_decimation_x", default_value="4"),
            DeclareLaunchArgument("down_camera_decimation_y", default_value="4"),
            DeclareLaunchArgument("down_camera_offset_x", default_value="0"),
            DeclareLaunchArgument("down_camera_offset_y", default_value="0"),
            DeclareLaunchArgument("down_camera_width", default_value="1280"),
            DeclareLaunchArgument("down_camera_height", default_value="720"),
            DeclareLaunchArgument(
                "front_camera_image_topic",
                default_value="/cirtesub/front_camera/camera/image_color",
            ),
            DeclareLaunchArgument(
                "front_camera_camera_info_topic",
                default_value="/cirtesub/front_camera/camera/camera_info",
            ),
            DeclareLaunchArgument(
                "front_camera_decimated_topic",
                default_value="/cirtesub/front_camera/decimated/image_raw",
            ),
            DeclareLaunchArgument(
                "front_camera_decimated_camera_info_topic",
                default_value="/cirtesub/front_camera/decimated/camera_info",
            ),
            DeclareLaunchArgument("front_camera_decimation_x", default_value="4"),
            DeclareLaunchArgument("front_camera_decimation_y", default_value="4"),
            DeclareLaunchArgument("front_camera_offset_x", default_value="0"),
            DeclareLaunchArgument("front_camera_offset_y", default_value="0"),
            DeclareLaunchArgument("front_camera_width", default_value="1280"),
            DeclareLaunchArgument("front_camera_height", default_value="720"),
            compressed_republish("down_camera", down_image),
            ComposableNodeContainer(
                name="sura_down_camera_proc_container",
                namespace="",
                package="rclcpp_components",
                executable="component_container",
                composable_node_descriptions=[
                    crop_decimate(
                        "down_camera",
                        down_image,
                        down_info,
                        down_decimated,
                        down_decimated_info,
                    )
                ],
                output="screen",
            ),
            compressed_republish("down_camera", down_decimated, "_decimated"),
            Node(
                condition=IfCondition(LaunchConfiguration("launch_down_aruco")),
                package="aruco_opencv",
                executable="aruco_tracker_autostart",
                namespace="/cirtesub/down_camera",
                name="aruco_tracker",
                output="screen",
                parameters=[aruco_tracker_config],
            ),
            Node(
                condition=IfCondition(LaunchConfiguration("launch_down_aruco")),
                package="image_transport",
                executable="republish",
                name="down_camera_aruco_debug_compressed_republish",
                output="screen",
                arguments=["raw", "compressed"],
                remappings=[
                    ("in", "/cirtesub/down_camera/aruco_tracker/debug"),
                    ("in/image_raw", "/cirtesub/down_camera/aruco_tracker/debug"),
                    ("out", "/cirtesub/down_camera/aruco_tracker/debug"),
                    (
                        "out/compressed",
                        "/cirtesub/down_camera/aruco_tracker/debug/compressed",
                    ),
                    ("out/image_raw", "/cirtesub/down_camera/aruco_tracker/debug"),
                    (
                        "out/image_raw/compressed",
                        "/cirtesub/down_camera/aruco_tracker/debug/compressed",
                    ),
                ],
            ),
            Node(
                condition=IfCondition(LaunchConfiguration("launch_front_camera")),
                package="image_transport",
                executable="republish",
                name="front_camera_compressed_republish",
                output="screen",
                arguments=["raw", "compressed"],
                remappings=[
                    ("in", front_image),
                    ("in/image_raw", front_image),
                    ("out", front_image),
                    ("out/compressed", [front_image, "/compressed"]),
                    ("out/image_raw", front_image),
                    ("out/image_raw/compressed", [front_image, "/compressed"]),
                ],
            ),
            ComposableNodeContainer(
                condition=IfCondition(LaunchConfiguration("launch_front_camera")),
                name="sura_front_camera_proc_container",
                namespace="",
                package="rclcpp_components",
                executable="component_container",
                composable_node_descriptions=[
                    crop_decimate(
                        "front_camera",
                        front_image,
                        front_info,
                        front_decimated,
                        front_decimated_info,
                    )
                ],
                output="screen",
            ),
            Node(
                condition=IfCondition(LaunchConfiguration("launch_front_camera")),
                package="image_transport",
                executable="republish",
                name="front_camera_decimated_compressed_republish",
                output="screen",
                arguments=["raw", "compressed"],
                remappings=[
                    ("in", front_decimated),
                    ("in/image_raw", front_decimated),
                    ("out", front_decimated),
                    ("out/compressed", [front_decimated, "/compressed"]),
                    ("out/image_raw", front_decimated),
                    ("out/image_raw/compressed", [front_decimated, "/compressed"]),
                ],
            ),
        ]
    )
