from setuptools import find_packages, setup

package_name = 'rov_state_bridge'

setup(
    name=package_name,
    version='0.1.0',
    packages=find_packages(where='src'),
    package_dir={'': 'src'},
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name + '/launch', ['launch/stage1_preview.launch.py']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='wys',
    maintainer_email='wys@example.com',
    description='Read-only SHM to ROS2 bridge and advisory health monitor for UnderwaterRobotSystem.',
    license='MIT',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'rov_state_bridge = rov_state_bridge.cli:main',
            'rov_health_monitor = rov_state_bridge.ros2_health_node:main',
        ],
    },
)
