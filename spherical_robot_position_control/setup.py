from setuptools import find_packages, setup

package_name = 'spherical_robot_position_control'

setup(
    name=package_name,
    version='0.1.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name + '/config', ['config/pose_bridge.yaml']),
        ('share/' + package_name + '/launch', ['launch/position_control.launch.py']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='gontary',
    maintainer_email='gloyejd477@gmail.com',
    description='Goal-to-(x,y) controller combining roll and pitch reaction-wheel control with drive limiting.',
    license='BSD-3-Clause',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'position_controller = spherical_robot_position_control.nodes.position_controller_node:main',
            'trajectory_monitor = spherical_robot_position_control.nodes.trajectory_monitor:main',
        ],
    },
)
