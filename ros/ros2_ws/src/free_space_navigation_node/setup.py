from setuptools import setup

package_name = 'free_space_navigation_node'

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
    description='Refined navigation node using semantic-depth free-space status',
    license='Apache-2.0',
    entry_points={
        'console_scripts': [
            'free_space_navigation_node = free_space_navigation_node.free_space_navigation_node:main',
        ],
    },
)
