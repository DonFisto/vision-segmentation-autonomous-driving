from setuptools import find_packages, setup

package_name = "planning_visualization"

setup(
    name=package_name,
    version="0.0.0",
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
    ],
    install_requires=["setuptools"],
    zip_safe=True,
    maintainer="danielmartinez",
    maintainer_email="daniel.mcvg@gmail.com",
    description="Diagnostic visualization nodes for the planning stack.",
    license="TODO",
    tests_require=["pytest"],
    entry_points={
        "console_scripts": [
            "route_plan_visualizer_node = "
            "planning_visualization.route_plan_visualizer_node:main",
            "odom_tf_broadcaster_node = "
            "planning_visualization.odom_tf_broadcaster_node:main",
            "global_lane_graph_visualizer_node = "
            "planning_visualization.global_lane_graph_visualizer_node:main",
        ],
    },
)
