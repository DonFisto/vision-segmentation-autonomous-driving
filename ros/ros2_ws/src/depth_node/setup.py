from setuptools import setup

package_name = 'depth_node'

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
    description='Depth Anything ROS2 inference node',
    license='Apache-2.0',
    entry_points={
        'console_scripts': [
            'depth_node = depth_node.depth_node:main',
        ],
    },
)
