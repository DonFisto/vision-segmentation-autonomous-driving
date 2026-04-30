from setuptools import setup

package_name = 'free_space_node'

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
    description='Semantic-depth free-space estimation node',
    license='Apache-2.0',
    entry_points={
        'console_scripts': [
            'free_space_node = free_space_node.free_space_node:main',
        ],
    },
)
