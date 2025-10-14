from setuptools import setup, find_packages

package_name = 'spherical_robot_pitch_control'

setup(
    name=package_name,
    version='0.1.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name + '/launch', ['launch/pitch_control.launch.py']),
        ('share/' + package_name + '/config', ['config/bridge.yaml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='gontary',
    maintainer_email='gloyejd477@gmail.com',
    description='Pitch stabilizer using reaction wheels and Gazebo pose info',
    license='BSD-3-Clause',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'pitch_stabilizer = spherical_robot_pitch_control.pitch_stabilizer:main',
        ],
    },
)

