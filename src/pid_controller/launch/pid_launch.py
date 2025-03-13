import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

def generate_launch_description():
    # Declare launch arguments
    kp = DeclareLaunchArgument('kp', default_value='0.01')
    ki = DeclareLaunchArgument('ki', default_value='0.0')
    kd = DeclareLaunchArgument('kd', default_value='0.0')

    # Get the path to the parameters.yaml file
    config = os.path.join(
        get_package_share_directory('pid_controller'),
        'config',
        'parameters.yaml'
    )

    # Define nodes
    pid_controller_node = Node(
        package='pid_controller',
        executable='runPID',
        name='pid_controller',
        namespace='namespace',
        parameters=[
            {'kp': LaunchConfiguration('kp')},
            {'ki': LaunchConfiguration('ki')},
            {'kd': LaunchConfiguration('kd')}
        ]
    )

    joint_simulator_node = Node(
        package='joint_simulator',
        executable='RunJointSim',
        name='joint_simulator',
        namespace='namespace',
        parameters=[config]
    )

    reference_input_node = Node(
        package='pid_controller',
        executable='client',
        name='reference_input',
        namespace='namespace'
    )

    # Create and return launch description
    return LaunchDescription([
        kp,
        ki,
        kd,
        pid_controller_node,
        joint_simulator_node,
        reference_input_node
    ])

