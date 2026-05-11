import os
import tempfile

import yaml
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import ComposableNodeContainer, Node
from launch_ros.descriptions import ComposableNode
from launch_ros.parameter_descriptions import ParameterValue


def bool_launch_arg(context, name, default_value):
    value = LaunchConfiguration(name).perform(context).lower()
    if value == "auto":
        value = default_value

    if value in ("true", "1", "yes", "on"):
        return True
    if value in ("false", "0", "no", "off"):
        return False

    raise RuntimeError(
        f"Unsupported value '{value}' for launch argument '{name}'. "
        "Use true, false, or auto."
    )


def robot_namespace(context):
    namespace = LaunchConfiguration("robot_namespace").perform(context).strip("/")
    if not namespace:
        raise RuntimeError("Launch argument 'robot_namespace' cannot be empty.")
    return namespace


def topic(namespace, path):
    return f"/{namespace}/{path}"


def replace_robot_namespace(value, namespace):
    if isinstance(value, str) and value.startswith("/cirtesub/"):
        return f"/{namespace}/{value[len('/cirtesub/'):]}"
    return value


def namespace_camera_config(camera_cfg, namespace):
    return {
        key: replace_robot_namespace(value, namespace)
        for key, value in camera_cfg.items()
    }


def namespaced_aruco_config(config_path, camera_image_topic, namespace, suffix):
    with open(config_path, "r") as f:
        config = yaml.safe_load(f)

    config["/**"]["ros__parameters"]["cam_base_topic"] = camera_image_topic

    safe_namespace = namespace.replace("/", "_")
    output_path = os.path.join(
        tempfile.gettempdir(),
        f"sura_cameras_{safe_namespace}_{suffix}_aruco.yaml",
    )

    with open(output_path, "w") as f:
        yaml.safe_dump(config, f, sort_keys=False)

    return output_path


def compressed_republish(camera_name, image_topic, suffix=""):
    return Node(
        package="image_transport",
        executable="republish",
        name=f"{camera_name}{suffix}_compressed_republish",
        output="log",
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


def sim_crop_decimate(
    camera_name,
    image_topic,
    camera_info_topic,
    decimated_topic,
    decimated_info_topic,
):
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


def load_camera_calibration(camera_name, camera_cfg, calibrations_config):
    if camera_name not in calibrations_config:
        return None

    camera_calibs = calibrations_config[camera_name]

    if not camera_calibs.get("calibrated", False):
        return None

    width = int(camera_cfg["width"])
    height = int(camera_cfg["height"])
    calibration_key = f"calibration_{width}x{height}"

    if calibration_key not in camera_calibs:
        return None

    calibration_data = calibrations_config[camera_name][calibration_key]
    calibration_path = os.path.join(
        tempfile.gettempdir(),
        f"{camera_name}_{width}x{height}_calib.yaml",
    )

    with open(calibration_path, "w") as f:
        yaml.safe_dump(calibration_data, f, sort_keys=False)

    return f"file://{calibration_path}"


def usb_camera_node(camera_name, camera_cfg):
    parameters = {
        "video_device": camera_cfg["device"],
        "camera_name": camera_cfg.get("camera_name", camera_name),
        "image_width": int(camera_cfg["width"]),
        "image_height": int(camera_cfg["height"]),
        "framerate": float(camera_cfg.get("framerate", 15.0)),
        "pixel_format": camera_cfg.get("pixel_format", "mjpeg2rgb"),
        "io_method": camera_cfg.get("io_method", "mmap"),
        "camera_frame_id": camera_cfg.get("frame_id", camera_name),
    }

    if camera_cfg.get("camera_info_url"):
        parameters["camera_info_url"] = camera_cfg["camera_info_url"]

    return Node(
        package="usb_cam",
        executable="usb_cam_node_exe",
        name=f"{camera_name}_usb_cam",
        output="log",
        parameters=[parameters],
        remappings=[
            ("image_raw", camera_cfg["image_topic"]),
            ("camera_info", camera_cfg["camera_info_topic"]),
        ],
    )


def real_crop_decimate(camera_name, camera_cfg):
    return ComposableNodeContainer(
        name=f"sura_{camera_name}_proc_container",
        namespace="",
        package="rclcpp_components",
        executable="component_container",
        output="log",
        composable_node_descriptions=[
            ComposableNode(
                package="image_proc",
                plugin="image_proc::CropDecimateNode",
                name=f"{camera_name}_crop_decimate",
                parameters=[
                    {
                        "decimation_x": int(camera_cfg["decimation_x"]),
                        "decimation_y": int(camera_cfg["decimation_y"]),
                        "offset_x": int(camera_cfg["offset_x"]),
                        "offset_y": int(camera_cfg["offset_y"]),
                        "width": int(camera_cfg["width"]),
                        "height": int(camera_cfg["height"]),
                    }
                ],
                remappings=[
                    ("in/image", camera_cfg["image_topic"]),
                    ("in/image_raw", camera_cfg["image_topic"]),
                    ("in/camera_info", camera_cfg["camera_info_topic"]),
                    ("out/image", camera_cfg["decimated_topic"]),
                    ("out/image_raw", camera_cfg["decimated_topic"]),
                    ("out/camera_info", camera_cfg["decimated_camera_info_topic"]),
                ],
            )
        ],
    )


def rectify(camera_name, camera_cfg):
    return ComposableNodeContainer(
        name=f"sura_{camera_name}_rectify_container",
        namespace="",
        package="rclcpp_components",
        executable="component_container",
        output="log",
        composable_node_descriptions=[
            ComposableNode(
                package="image_proc",
                plugin="image_proc::RectifyNode",
                name=f"{camera_name}_rectify",
                parameters=[{"queue_size": 5}],
                remappings=[
                    ("image", camera_cfg["image_topic"]),
                    ("image_raw", camera_cfg["image_topic"]),
                    ("camera_info", camera_cfg["camera_info_topic"]),
                    ("image_rect", camera_cfg["rectified_topic"]),
                    ("image_rect_color", camera_cfg["rectified_topic"]),
                ],
            )
        ],
    )


def aruco_down_node(aruco_tracker_config, namespace):
    return Node(
        package="aruco_opencv",
        executable="aruco_tracker_autostart",
        namespace=topic(namespace, "down_camera"),
        name="aruco_tracker",
        output="log",
        parameters=[aruco_tracker_config],
    )


def aruco_debug_compressed(namespace):
    debug_topic = topic(namespace, "down_camera/aruco_tracker/debug")

    return Node(
        package="image_transport",
        executable="republish",
        name="down_camera_aruco_debug_compressed_republish",
        output="log",
        arguments=["raw", "compressed"],
        remappings=[
            ("in", debug_topic),
            ("in/image_raw", debug_topic),
            ("out", debug_topic),
            ("out/compressed", f"{debug_topic}/compressed"),
            ("out/image_raw", debug_topic),
            ("out/image_raw/compressed", f"{debug_topic}/compressed"),
        ],
    )


def sim_launch_setup(context, camera_share):
    namespace = robot_namespace(context)

    down_image = LaunchConfiguration("down_camera_image_topic").perform(context)
    down_info = LaunchConfiguration("down_camera_camera_info_topic").perform(context)
    down_decimated = LaunchConfiguration("down_camera_decimated_topic").perform(context)
    down_decimated_info = LaunchConfiguration(
        "down_camera_decimated_camera_info_topic"
    ).perform(context)
    front_image = LaunchConfiguration("front_camera_image_topic").perform(context)
    front_info = LaunchConfiguration("front_camera_camera_info_topic").perform(context)
    front_decimated = LaunchConfiguration("front_camera_decimated_topic").perform(context)
    front_decimated_info = LaunchConfiguration(
        "front_camera_decimated_camera_info_topic"
    ).perform(context)

    aruco_tracker_config = namespaced_aruco_config(
        os.path.join(camera_share, "config", "aruco_tracker_down.yaml"),
        down_image,
        namespace,
        "sim",
    )

    launch_items = []

    if bool_launch_arg(context, "launch_down_camera", "true"):
        launch_items.extend(
            [
                compressed_republish("down_camera", down_image),
                ComposableNodeContainer(
                    name="sura_down_camera_proc_container",
                    namespace="",
                    package="rclcpp_components",
                    executable="component_container",
                    composable_node_descriptions=[
                        sim_crop_decimate(
                            "down_camera",
                            down_image,
                            down_info,
                            down_decimated,
                            down_decimated_info,
                        )
                    ],
                    output="log",
                ),
                compressed_republish("down_camera", down_decimated, "_decimated"),
            ]
        )

    if bool_launch_arg(context, "launch_down_aruco", "true"):
        launch_items.extend(
            [
                aruco_down_node(aruco_tracker_config, namespace),
                aruco_debug_compressed(namespace),
            ]
        )

    if bool_launch_arg(context, "launch_front_camera", "true"):
        launch_items.extend(
            [
                compressed_republish("front_camera", front_image),
                ComposableNodeContainer(
                    name="sura_front_camera_proc_container",
                    namespace="",
                    package="rclcpp_components",
                    executable="component_container",
                    composable_node_descriptions=[
                        sim_crop_decimate(
                            "front_camera",
                            front_image,
                            front_info,
                            front_decimated,
                            front_decimated_info,
                        )
                    ],
                    output="log",
                ),
                compressed_republish("front_camera", front_decimated, "_decimated"),
            ]
        )

    return launch_items


def real_camera_pipeline(camera_name, camera_cfg):
    return [
        usb_camera_node(camera_name, camera_cfg),
        compressed_republish(camera_name, camera_cfg["image_topic"]),
        real_crop_decimate(camera_name, camera_cfg),
        rectify(camera_name, camera_cfg),
    ]


def real_launch_setup(context, camera_share):
    namespace = robot_namespace(context)
    cameras_config_path = os.path.join(camera_share, "config", "cameras_real.yaml")
    calibrations_config_path = os.path.join(
        camera_share,
        "config",
        "camera_calibrations.yaml",
    )
    aruco_tracker_config = os.path.join(
        camera_share,
        "config",
        "aruco_tracker_down_real.yaml",
    )

    with open(cameras_config_path, "r") as f:
        cameras_config = yaml.safe_load(f)

    with open(calibrations_config_path, "r") as f:
        calibrations_config = yaml.safe_load(f)

    for camera_name, camera_cfg in cameras_config.items():
        camera_cfg = namespace_camera_config(camera_cfg, namespace)
        camera_cfg["camera_info_url"] = load_camera_calibration(
            camera_name,
            camera_cfg,
            calibrations_config,
        )
        cameras_config[camera_name] = camera_cfg

    aruco_tracker_config = namespaced_aruco_config(
        aruco_tracker_config,
        cameras_config["down_camera"]["image_topic"],
        namespace,
        "real",
    )

    launch_items = []

    if (
        bool_launch_arg(context, "launch_down_camera", "true")
        and "down_camera" in cameras_config
    ):
        launch_items.extend(
            real_camera_pipeline("down_camera", cameras_config["down_camera"])
        )

    if bool_launch_arg(context, "launch_down_aruco", "true"):
        launch_items.extend(
            [
                aruco_down_node(aruco_tracker_config, namespace),
                aruco_debug_compressed(namespace),
            ]
        )

    if (
        bool_launch_arg(context, "launch_front_camera", "true")
        and "front_camera" in cameras_config
    ):
        launch_items.extend(
            real_camera_pipeline("front_camera", cameras_config["front_camera"])
        )

    if (
        bool_launch_arg(context, "launch_left_camera", "true")
        and "left_camera" in cameras_config
    ):
        launch_items.extend(
            real_camera_pipeline("left_camera", cameras_config["left_camera"])
        )

    return launch_items


def launch_setup(context, *args, **kwargs):
    environment = LaunchConfiguration("environment").perform(context)
    camera_share = get_package_share_directory("sura_cameras")

    if environment == "sim":
        return sim_launch_setup(context, camera_share)
    if environment == "real":
        return real_launch_setup(context, camera_share)

    raise RuntimeError(
        f"Unsupported environment '{environment}'. Use 'sim' or 'real'."
    )


def generate_launch_description():
    return LaunchDescription(
        [
            DeclareLaunchArgument(
                "robot_namespace",
                default_value="bluerov",
                description="Robot namespace used for camera topics",
            ),
            DeclareLaunchArgument(
                "environment",
                default_value="sim",
                description="Execution environment: sim or real",
            ),
            DeclareLaunchArgument("launch_down_camera", default_value="true"),
            DeclareLaunchArgument("launch_front_camera", default_value="true"),
            DeclareLaunchArgument("launch_left_camera", default_value="true"),
            DeclareLaunchArgument(
                "launch_down_aruco",
                default_value="true",
                description="Use true/false, or auto: true in sim and real",
            ),
            DeclareLaunchArgument(
                "down_camera_image_topic",
                default_value=[
                    "/",
                    LaunchConfiguration("robot_namespace"),
                    "/down_camera/camera/image_color",
                ],
            ),
            DeclareLaunchArgument(
                "down_camera_camera_info_topic",
                default_value=[
                    "/",
                    LaunchConfiguration("robot_namespace"),
                    "/down_camera/camera/camera_info",
                ],
            ),
            DeclareLaunchArgument(
                "down_camera_decimated_topic",
                default_value=[
                    "/",
                    LaunchConfiguration("robot_namespace"),
                    "/down_camera/decimated/image_raw",
                ],
            ),
            DeclareLaunchArgument(
                "down_camera_decimated_camera_info_topic",
                default_value=[
                    "/",
                    LaunchConfiguration("robot_namespace"),
                    "/down_camera/decimated/camera_info",
                ],
            ),
            DeclareLaunchArgument("down_camera_decimation_x", default_value="4"),
            DeclareLaunchArgument("down_camera_decimation_y", default_value="4"),
            DeclareLaunchArgument("down_camera_offset_x", default_value="0"),
            DeclareLaunchArgument("down_camera_offset_y", default_value="0"),
            DeclareLaunchArgument("down_camera_width", default_value="1280"),
            DeclareLaunchArgument("down_camera_height", default_value="720"),
            DeclareLaunchArgument(
                "front_camera_image_topic",
                default_value=[
                    "/",
                    LaunchConfiguration("robot_namespace"),
                    "/front_camera/camera/image_color",
                ],
            ),
            DeclareLaunchArgument(
                "front_camera_camera_info_topic",
                default_value=[
                    "/",
                    LaunchConfiguration("robot_namespace"),
                    "/front_camera/camera/camera_info",
                ],
            ),
            DeclareLaunchArgument(
                "front_camera_decimated_topic",
                default_value=[
                    "/",
                    LaunchConfiguration("robot_namespace"),
                    "/front_camera/decimated/image_raw",
                ],
            ),
            DeclareLaunchArgument(
                "front_camera_decimated_camera_info_topic",
                default_value=[
                    "/",
                    LaunchConfiguration("robot_namespace"),
                    "/front_camera/decimated/camera_info",
                ],
            ),
            DeclareLaunchArgument("front_camera_decimation_x", default_value="4"),
            DeclareLaunchArgument("front_camera_decimation_y", default_value="4"),
            DeclareLaunchArgument("front_camera_offset_x", default_value="0"),
            DeclareLaunchArgument("front_camera_offset_y", default_value="0"),
            DeclareLaunchArgument("front_camera_width", default_value="1280"),
            DeclareLaunchArgument("front_camera_height", default_value="720"),
            OpaqueFunction(function=launch_setup),
        ]
    )
