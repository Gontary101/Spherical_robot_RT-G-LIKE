import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
  pkg_share = get_package_share_directory('spherical_robot_pitch_control')
  bridge_yaml = os.path.join(pkg_share, 'config', 'bridge.yaml')

  pose_bridge = Node(
      package='ros_gz_bridge',
      executable='parameter_bridge',
      parameters=[{'config_file': bridge_yaml}],
      output='screen')

  stabilizer = Node(
      package='spherical_robot_pitch_control',
      executable='pitch_stabilizer',
      output='screen',
      parameters=[{
          'entity_name': 'spherical_robot',
          'link_name': 'frame',
          'kp': 10.0,
          'kd': 2.0,
          'max_torque': 50.0,
          'w_sign': -1.0,
          'pose_topic': '/world/default/pose/info',
          'effort_command_topic': '/spherical_robot/inertial_wheel_effort_controller/commands',
      }])

  return LaunchDescription([pose_bridge, stabilizer])

