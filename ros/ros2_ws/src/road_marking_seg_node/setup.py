from setuptools import find_packages, setup


package_name = "road_marking_seg_node"


setup(
    name=package_name,
    version="0.1.0",
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
        "Binary road-marking segmentation node using "
        "MMSegmentation."
    ),
    license="MIT",
    entry_points={
        "console_scripts": [
            (
                "road_marking_node = "
                "road_marking_seg_node.road_marking_node:main"
            ),
        ],
    },
)
