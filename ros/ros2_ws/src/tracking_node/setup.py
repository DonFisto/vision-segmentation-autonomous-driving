from setuptools import setup

package_name = 'tracking_node'

setup(
    name=package_name,
    version='0.0.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages',
         ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='danielmartinez',
    maintainer_email='you@email.com',
    description='Simple tracking-by-detection node',
    license='Apache License 2.0',
    entry_points={
        'console_scripts': [
            'tracking_node = tracking_node.tracking_node:main',
        ],
    },
)
