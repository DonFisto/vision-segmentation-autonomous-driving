from setuptools import find_packages, setup

package_name = "lane_dataset_recorder"

setup(
    name=package_name,
    version="0.1.0",
    packages=find_packages(exclude=["test"]),
    data_files=[
        (
            "share/ament_index/resource_index/packages",
            [f"resource/{package_name}"],
        ),
        (
            f"share/{package_name}",
            ["package.xml"],
        ),
    ],
    install_requires=["setuptools"],
    zip_safe=True,
    maintainer="danielmartinez",
    maintainer_email="danielmartinez@todo.todo",
    description=(
        "Synchronized CARLA RGB and semantic road-line dataset recorder."
    ),
    license="MIT",
    entry_points={
        "console_scripts": [
            (
                "lane_dataset_recorder = "
                "lane_dataset_recorder.lane_dataset_recorder:main"
            ),
        ],
    },
)
