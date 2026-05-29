import os

import yaml
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, OpaqueFunction
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration


def enabled(value):
    return str(value).lower() in ("true", "1", "yes", "on")


def launch_setup(context, *args, **kwargs):
    cameras = yaml.safe_load(LaunchConfiguration("cameras").perform(context)) or {}
    cameras_share = get_package_share_directory("sura_cameras")
    single_camera_launch = os.path.join(cameras_share, "launch", "single_camera.launch.py")

    launches = []
    for camera_name, camera in cameras.items():
        camera = camera or {}
        if not enabled(camera.get("enabled", False)):
            continue

        camera_config_dir = os.path.join(cameras_share, "config", camera_name)
        if not os.path.isdir(camera_config_dir):
            continue

        launches.append(
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(single_camera_launch),
                launch_arguments=[
                    ("camera_name", camera_name),
                    ("camera_config_dir", camera_config_dir),
                    ("environment", LaunchConfiguration("environment")),
                    ("aruco", str(enabled(camera.get("aruco", False))).lower()),
                ],
            )
        )

    return launches


def generate_launch_description():
    return LaunchDescription(
        [
            DeclareLaunchArgument("environment", default_value="sim"),
            DeclareLaunchArgument("cameras", default_value="{}"),
            OpaqueFunction(function=launch_setup),
        ]
    )
