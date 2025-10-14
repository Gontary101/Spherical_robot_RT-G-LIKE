import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess, SetEnvironmentVariable, TimerAction
from launch.conditions import IfCondition, UnlessCondition
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
  package_share = get_package_share_directory('spherical_robot_control')
  default_world = os.path.join(package_share, 'worlds', 'spherical_world.sdf')
  urdf_path = os.path.join(package_share, 'urdf', 'spherical_robot.urdf')
  models_path = os.path.join(os.path.expanduser('~'), 'ros_ws', 'src', 'gz_models')

  with open(urdf_path, 'r', encoding='utf-8') as urdf_file:
    robot_description_content = urdf_file.read()

  existing_ign_resource = os.environ.get('IGN_GAZEBO_RESOURCE_PATH', '')
  ign_resource_path = (
      f"{models_path}:{existing_ign_resource}"
      if existing_ign_resource else models_path)

  existing_gz_resource = os.environ.get('GZ_SIM_RESOURCE_PATH', '')
  gz_resource_path = (
      f"{models_path}:{existing_gz_resource}"
      if existing_gz_resource else models_path)

  world_arg = DeclareLaunchArgument(
      'world',
      default_value=default_world,
      description='Path to the Gazebo world SDF file')

  gui_arg = DeclareLaunchArgument(
      'gui', default_value='true',
      description='Whether to run Gazebo with the GUI')

  start_sim_gui = ExecuteProcess(
      cmd=['ign', 'gazebo', LaunchConfiguration('world'), '-r', '-v', '4'],
      output='screen',
      condition=IfCondition(LaunchConfiguration('gui')))

  start_sim_headless = ExecuteProcess(
      cmd=['ign', 'gazebo', LaunchConfiguration('world'), '-r', '-v', '4', '-s'],
      output='screen',
      condition=UnlessCondition(LaunchConfiguration('gui')))

  robot_state_publisher = Node(
      package='robot_state_publisher',
      executable='robot_state_publisher',
      namespace='spherical_robot',
      parameters=[{'robot_description': robot_description_content}],
      output='screen')

  joint_state_spawner = Node(
      package='controller_manager',
      executable='spawner',
      arguments=[
          'joint_state_broadcaster',
          '--controller-manager', '/spherical_robot/controller_manager'],
      output='screen')

  inertial_spawner = Node(
      package='controller_manager',
      executable='spawner',
      arguments=[
          'inertial_wheel_effort_controller',
          '--controller-manager', '/spherical_robot/controller_manager'],
      output='screen')

  drive_spawner = Node(
      package='controller_manager',
      executable='spawner',
      arguments=[
          'drive_motor_velocity_controller',
          '--controller-manager', '/spherical_robot/controller_manager'],
      output='screen')

  delayed_spawners = TimerAction(
      period=4.0,
      actions=[joint_state_spawner, inertial_spawner, drive_spawner])

  drive_bridge = Node(
      package='spherical_robot_control',
      executable='drive_command_bridge',
      namespace='spherical_robot',
      output='screen',
      parameters=[{
          'input_topic': 'drive_velocity_command',
          'controller_topic': '/spherical_robot/drive_motor_velocity_controller/commands',
          'outer_ratio': 0.25,
          'motor_2_sign': -1.0,
      }])

  delayed_drive_bridge = TimerAction(
      period=5.0,
      actions=[drive_bridge])

  bridge_config_path = os.path.join(package_share, 'config', 'spherical_robot_bridge.yaml')

  imu_bridge = Node(
      package='ros_gz_bridge',
      executable='parameter_bridge',
      arguments=['--ros-args', '-p', f'config_file:={bridge_config_path}'],
      output='screen')

  return LaunchDescription([
      world_arg,
      gui_arg,
      SetEnvironmentVariable('IGN_GAZEBO_RESOURCE_PATH', ign_resource_path),
      SetEnvironmentVariable('GZ_SIM_RESOURCE_PATH', gz_resource_path),
      start_sim_gui,
      start_sim_headless,
      robot_state_publisher,
      delayed_spawners,
      delayed_drive_bridge,
      imu_bridge,
  ])
