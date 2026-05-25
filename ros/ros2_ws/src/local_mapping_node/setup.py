from setuptools import setup

package_name = 'local_mapping_node'

setup(
    name=package_name,
    version='0.0.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='danielmartinez',
    maintainer_email='danielmartinez@example.com',
    description='Accumulated local mapping from local occupancy grids and CARLA hero odometry',
    license='Apache-2.0',
    entry_points={
        'console_scripts': [
            'local_mapping_node = local_mapping_node.local_mapping_node:main',
        ],
    },
)
