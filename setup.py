from glob import glob
import os

from setuptools import setup


package_name = "sura_cameras"


def package_files(directory):
    files = []
    for path in glob(os.path.join(directory, "**", "*.yaml"), recursive=True):
        install_dir = os.path.join("share", package_name, os.path.dirname(path))
        files.append((install_dir, [path]))
    return files


setup(
    name=package_name,
    version="0.0.0",
    packages=[package_name],
    data_files=[
        ("share/ament_index/resource_index/packages", ["resource/" + package_name]),
        ("share/" + package_name, ["package.xml"]),
        (os.path.join("share", package_name, "launch"), glob("launch/*.launch.py")),
    ]
    + package_files("config"),
    install_requires=["setuptools"],
    zip_safe=True,
    maintainer="usuario",
    maintainer_email="iedo@uji.es",
    description="Launch files for SURA camera image processing pipelines.",
    license="TODO: License declaration",
)
