from setuptools import find_packages, setup

package_name = "perception_geometry"

setup(
    name=package_name,
    version="0.1.0",
    packages=find_packages(exclude=["test"]),
    data_files=[
        ("share/ament_index/resource_index/packages", [f"resource/{package_name}"]),
        (f"share/{package_name}", ["package.xml"]),
    ],
    install_requires=["setuptools"],
    zip_safe=True,
    maintainer="Daniel Martínez-Cabeza de Vaca Guillén",
    maintainer_email="user@example.com",
    description="Shared camera geometry and IPM utilities for the ROS2/CARLA perception stack.",
    license="MIT",
    tests_require=["pytest"],
)
