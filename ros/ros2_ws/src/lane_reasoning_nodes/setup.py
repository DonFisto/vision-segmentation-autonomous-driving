from setuptools import setup

package_name = "lane_reasoning_nodes"

setup(
    name=package_name,
    version="0.1.0",
    packages=[package_name],
    data_files=[
        ("share/ament_index/resource_index/packages", ["resource/" + package_name]),
        ("share/" + package_name, ["package.xml"]),
    ],
    install_requires=["setuptools"],
    zip_safe=True,
    maintainer="Daniel Martinez",
    maintainer_email="maintainer@example.com",
    description="Lane projection, mapping, and guidance skeleton nodes.",
    license="MIT",
    entry_points={
        "console_scripts": [
            "lane_projection_node = lane_reasoning_nodes.lane_projection_node:main",
            "lane_mapping_node = lane_reasoning_nodes.lane_mapping_node:main",
            "lane_guidance_node = lane_reasoning_nodes.lane_guidance_node:main",
        ],
    },
)
