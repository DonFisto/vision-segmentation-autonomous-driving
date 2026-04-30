from setuptools import setup

package_name = 'reactive_navigation_node'

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
    description='Simple reactive navigation node using fused objects and relative depth',
    license='Apache-2.0',
    entry_points={
        'console_scripts': [
            'reactive_navigation_node = reactive_navigation_node.reactive_navigation_node:main',
        ],
    },
)
