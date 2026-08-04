from setuptools import find_packages, setup


package_name = "lane_geometry_node"


setup(
    name=package_name,
    version="0.2.0",
    packages=find_packages(exclude=["test"]),
    data_files=[
        (
            "share/ament_index/resource_index/packages",
            ["resource/" + package_name],
        ),
        (
            "share/" + package_name,
            ["package.xml"],
        ),
        (
            "share/" + package_name,
            ["LICENSE"],
        ),
    ],
    install_requires=["setuptools"],
    zip_safe=True,
    maintainer="danielmartinez",
    maintainer_email="daniel.mcvg@gmail.com",
    description=(
        "Metric BEV projection and lane-marking geometry processing."
    ),
    license="MIT",
    entry_points={
        "console_scripts": [
            (
                "lane_geometry_node = "
                "lane_geometry_node.lane_geometry_node:main"
            ),
            (
                "lane_component_filter_node = "
                "lane_geometry_node.lane_component_filter_node:main"
            ),
            (
                "lane_context_filter_node = "
                "lane_geometry_node.lane_context_filter_node:main"
            ),
            (
                "lane_curve_fit_node = "
                "lane_geometry_node.lane_curve_fit_node:main"
            ),

            (
                "lane_tracking_node = "
                "lane_geometry_node.lane_tracking_node:main"
            ),
        ],
    },
)
