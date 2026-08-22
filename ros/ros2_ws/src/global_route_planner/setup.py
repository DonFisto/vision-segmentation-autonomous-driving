from setuptools import setup


package_name = "global_route_planner"


setup(
    name=package_name,
    version="0.1.0",
    packages=[package_name],
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
    maintainer="Daniel Martinez",
    maintainer_email="daniel.mcvg@gmail.com",
    description=(
        "Global CARLA/OpenDRIVE topology extraction "
        "and route planning."
    ),
    license="MIT",
    entry_points={
        "console_scripts": [
            (
                "topology_diagnostic_node = "
                "global_route_planner."
                "topology_diagnostic_node:main"
            ),
            (
                "routing_diagnostic_node = "
                "global_route_planner."
                "routing_diagnostic_node:main"
            ),
            (
                "routing_graph_diagnostic_node = "
                "global_route_planner."
                "routing_graph_diagnostic_node:main"
            ),
            (
                "route_association_diagnostic_node = "
                "global_route_planner."
                "route_association_diagnostic_node:main"
            ),
        ],
    },
)
