from setuptools import setup

package_name = "lane_detection_node"

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
    description="Classical lane detection node using RGB images and semantic road ROI.",
    license="MIT",
    tests_require=["pytest"],
    entry_points={
        "console_scripts": [
            "lane_detection_node = lane_detection_node.lane_detection_node:main",
        ],
    },
)
