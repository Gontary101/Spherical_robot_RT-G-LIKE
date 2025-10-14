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
          'use_imu': True,
          'imu_topic': '/spherical_robot/frame_imu',
          # Initial gains (good first pass; tune live with rqt_reconfigure or by relaunch)
          'kp': 12.0,
          'ki': 4.0,
          'kd': 3.0,
          'max_torque': 50.0,
          # IMPORTANT: mirrored wheel gets the opposite command so torques add on the frame
          'w_sign': 1.0,
          'target_pitch_rad': 0.0,
          'deadband_rad': 0.02,
          'rate_lpf_alpha': 0.2,
          'i_limit': 20.0,
          # pose_topic is unused when use_imu=True, but we keep it configurable
          'pose_topic': '/world/default/pose/info',
          'effort_command_topic': '/spherical_robot/inertial_wheel_effort_controller/commands',
      }])

  return LaunchDescription([pose_bridge, stabilizer])
