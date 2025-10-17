import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description():
  pkg_share = get_package_share_directory('spherical_robot_position_control')
  bridge_yaml = os.path.join(pkg_share, 'config', 'pose_bridge.yaml')

  goal_x_arg = DeclareLaunchArgument('goal_x', default_value='0.0')
  goal_y_arg = DeclareLaunchArgument('goal_y', default_value='0.0')
  motor_rad_max_arg = DeclareLaunchArgument('motor_rad_max', default_value='15.0')

  pose_bridge = Node(
      package='ros_gz_bridge',
      executable='parameter_bridge',
      parameters=[{'config_file': bridge_yaml}],
      output='screen')

  controller = Node(
      package='spherical_robot_position_control',
      executable='position_controller',
      output='screen',
      parameters=[{
          'imu_topic': '/spherical_robot/frame_imu',
          'pose_topic': '/world/default/pose/info',
          'joint_states_topic': '/joint_states',
          'roll_effort_topic': '/spherical_robot/inertial_wheel_roll_effort_controller/commands',
          'pitch_effort_topic': '/spherical_robot/inertial_wheel_pitch_effort_controller/commands',
          'drive_command_topic': '/spherical_robot/drive_velocity_command',
          'goal_x': ParameterValue(LaunchConfiguration('goal_x'), value_type=float),
          'goal_y': ParameterValue(LaunchConfiguration('goal_y'), value_type=float),
          'stop_distance': 0.5,
          'motor_rad_max': ParameterValue(LaunchConfiguration('motor_rad_max'), value_type=float),
          'k_motor': 0.7,
          'k_heading': 2.5,
          'k_heading_rate_damp': 0.6,
          'heading_gate_rad': 1.0472,
          'gravity': 9.81,
          'roll_max_rad': 0.2618,
          'friction_mu_max': 0.8,
          'roll_enable_shell_rad': 0.5,
          'creep_motor_rad': 0.8,
          'motor_rad_acc_limit': 3.0,
          'drive_wheel_radius': 0.26926,
          'outer_velocity_ratio': 0.25,
          'motor_2_sign': -1.0,
          'kp_roll': 5.0,
          'ki_roll': 1.0,
          'kd_roll': 3.0,
          'kp_pitch': 10.0,
          'ki_pitch': 3.0,
          'kd_pitch': 4.0,
          'rate_lpf_alpha': 0.2,
          'deadband_rad': 0.02,
          'torque_limit_nm': 30.0,
          'i_limit_nm': 10.0,
          'gyro_ff_enable': True,
          'gyro_ff_gain': 1.0,
          'wheel_inertia_kgm2': 0.0530374,
          'roll_pair_sign': 1.0,
          'pitch_pair_sign': 1.0,
          'control_rate_hz': 200.0,
          'log_period': 0.5,
      }])

  monitor = Node(
      package='spherical_robot_position_control',
      executable='trajectory_monitor',
      output='screen',
      parameters=[{
          'pose_topic': '/world/default/pose/info',
          'joint_states_topic': '/joint_states',
          'drive_command_topic': '/spherical_robot/drive_velocity_command',
          'goal_x': ParameterValue(LaunchConfiguration('goal_x'), value_type=float),
          'goal_y': ParameterValue(LaunchConfiguration('goal_y'), value_type=float),
          'entity_name': 'spherical_robot',
          'link_name': 'frame',
          'drive_wheel_radius': 0.26926,
          'outer_velocity_ratio': 0.25,
          'motor_2_sign': -1.0,
          'log_period': 0.5,
      }])

  return LaunchDescription([goal_x_arg, goal_y_arg, motor_rad_max_arg, pose_bridge, controller, monitor])
