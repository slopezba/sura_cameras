import os
import tempfile

# Camera pipeline launch description.

import yaml
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess, OpaqueFunction
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def enabled(value):
    return str(value).lower() in ("true", "1", "yes", "on")


def load_yaml(path):
    if not os.path.exists(path):
        return {}
    with open(path, "r", encoding="utf-8") as f:
        return yaml.safe_load(f) or {}


def camera_info_url(camera_name, camera):
    calibration = load_yaml(os.path.join(camera["config_dir"], "calibration.yaml"))
    key = f"calibration_{int(camera['width'])}x{int(camera['height'])}"
    if not calibration.get("calibrated", False) or key not in calibration:
        return ""

    calibration_path = os.path.join(
        tempfile.gettempdir(),
        f"{camera_name}_{int(camera['width'])}x{int(camera['height'])}_calibration.yaml",
    )
    with open(calibration_path, "w", encoding="utf-8") as f:
        yaml.safe_dump(calibration[key], f, sort_keys=False)

    return f"file://{calibration_path}"


def usb_camera(camera_name, camera):
    parameters = {
        "video_device": camera["port"],
        "camera_name": camera_name,
        "image_width": int(camera["width"]),
        "image_height": int(camera["height"]),
        "framerate": float(camera["framerate"]),
        "pixel_format": camera.get("pixel_format", "mjpeg2rgb"),
        "io_method": camera.get("io_method", "mmap"),
        "frame_id": camera.get("frame_id", camera_name),
        "camera_frame_id": camera.get("frame_id", camera_name),
    }

    url = camera_info_url(camera_name, camera)
    if url:
        parameters["camera_info_url"] = url

    return Node(
        package="usb_cam",
        executable="usb_cam_node_exe",
        namespace=f"{camera_name}/camera",
        name=f"{camera_name}_usb_cam",
        output="log",
        parameters=[parameters],
    )


def gstreamer_camera(camera_name, camera):
    gstreamer = camera.get("gstreamer", {}) or {}
    width = int(gstreamer.get("width", camera["width"]))
    height = int(gstreamer.get("height", camera["height"]))
    framerate = int(gstreamer.get("framerate", camera.get("framerate", 15)))
    host = str(gstreamer.get("host", "192.168.1.236"))
    port = int(gstreamer.get("port", 5600))
    bitrate = int(gstreamer.get("bitrate", 3500))
    key_int_max = int(gstreamer.get("key_int_max", framerate))
    threads = int(gstreamer.get("threads", 2))

    return ExecuteProcess(
        cmd=[
            "gst-launch-1.0",
            "-v",
            "v4l2src",
            f"device={camera['port']}",
            "do-timestamp=true",
            "!",
            f"image/jpeg,width={width},height={height},framerate={framerate}/1",
            "!",
            "jpegdec",
            "!",
            "videoconvert",
            "!",
            "x264enc",
            "tune=zerolatency",
            "speed-preset=ultrafast",
            f"bitrate={bitrate}",
            f"key-int-max={key_int_max}",
            f"threads={threads}",
            "!",
            "video/x-h264,profile=baseline",
            "!",
            "h264parse",
            "config-interval=1",
            "!",
            "rtph264pay",
            "pt=96",
            "config-interval=1",
            "!",
            "udpsink",
            f"host={host}",
            f"port={port}",
            "sync=false",
            "async=false",
        ],
        name=f"{camera_name}_gstreamer",
        output="screen",
    )


def sim_camera_republisher(camera_name, camera):
    parameters = {
        "input_image_topic": camera["stonefish_topic"],
        "input_compressed_topic": f"{camera['stonefish_topic']}/compressed",
        "camera_name": camera_name,
        "frame_id": camera.get("frame_id", camera_name),
    }

    url = camera_info_url(camera_name, camera)
    if url:
        parameters["camera_info_url"] = url

    return Node(
        package="sura_cameras",
        executable="sim_camera_republisher",
        namespace=f"{camera_name}/camera",
        name=f"{camera_name}_sim_camera_republisher",
        output="log",
        parameters=[parameters],
    )


def decimated(camera_name, camera, environment):
    decimation = camera.get("decimated", {})
    return Node(
        package="sura_cameras",
        executable="image_decimator",
        namespace=f"{camera_name}/camera",
        name=f"{camera_name}_image_decimator",
        output="log",
        remappings=[
            ("image_raw", "image_raw"),
            ("camera_info", "camera_info"),
        ],
        parameters=[
            {
                "decimation_x": int(decimation.get("decimation_x", 2)),
                "decimation_y": int(decimation.get("decimation_y", 2)),
                "offset_x": int(decimation.get("offset_x", 0)),
                "offset_y": int(decimation.get("offset_y", 0)),
                "width": int(decimation.get("width", int(camera["width"]))),
                "height": int(decimation.get("height", int(camera["height"]))),
            }
        ],
    )


def aruco_tracker(camera_name, camera, environment):
    aruco_config = os.path.join(camera["config_dir"], "aruco_tracker.yaml")
    parameters = [aruco_config]

    return Node(
        package="aruco_opencv",
        executable="aruco_tracker_autostart",
        namespace=camera_name,
        name="aruco_tracker",
        output="log",
        parameters=parameters,
    )


def launch_setup(context, *args, **kwargs):
    camera_name = LaunchConfiguration("camera_name").perform(context)
    config_dir = LaunchConfiguration("camera_config_dir").perform(context)
    environment = LaunchConfiguration("environment").perform(context)
    launch_aruco = enabled(LaunchConfiguration("aruco").perform(context))
    driver_override = LaunchConfiguration("driver").perform(context).strip()

    camera = load_yaml(os.path.join(config_dir, "camera.yaml"))
    if not camera:
        return []

    camera["config_dir"] = config_dir
    if driver_override:
        camera["driver"] = driver_override

    driver = str(camera.get("driver", "usb_cam")).lower()
    nodes = []

    if environment == "real":
        if driver == "gstreamer":
            nodes.append(gstreamer_camera(camera_name, camera))
        else:
            nodes.append(usb_camera(camera_name, camera))
    elif environment == "sim":
        nodes.append(sim_camera_republisher(camera_name, camera))

    if environment == "real" and driver != "usb_cam":
        return nodes

    if enabled(camera.get("decimated", {}).get("enabled", False)):
        nodes.append(decimated(camera_name, camera, environment))

    if launch_aruco and os.path.exists(os.path.join(config_dir, "aruco_tracker.yaml")):
        nodes.append(aruco_tracker(camera_name, camera, environment))

    return nodes


def generate_launch_description():
    return LaunchDescription(
        [
            DeclareLaunchArgument("camera_name"),
            DeclareLaunchArgument("camera_config_dir"),
            DeclareLaunchArgument("environment", default_value="sim"),
            DeclareLaunchArgument("aruco", default_value="false"),
            DeclareLaunchArgument("driver", default_value=""),
            OpaqueFunction(function=launch_setup),
        ]
    )
