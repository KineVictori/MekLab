from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch_ros.actions import Node
from launch.launch_description_sources import PythonLaunchDescriptionSource
from ament_index_python.packages import get_package_share_directory
import os
import xacro

def generatePythonLaunch(pkg_share):
    # Creates the qube_driver launch node.

    return IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                [os.path.join(pkg_share, "launch", "qube_driver.launch.py")]
            )
        )

def generateRViz(pkg_share):
    # Creates the rwiz launch node.

    return Node(
            package='rviz2',
            executable='rviz2',
            name='rviz2',
            output='screen',
            parameters=[{'use_sim_time': True}],
            arguments = ['d', [os.path.join(pkg_share, 'config', 'Config.rviz')]]
        )

def generateGUI():
    # Creates the GUI node.

    return Node(
            package='joint_state_publisher_gui',
            executable='joint_state_publisher_gui',
            name='joint_state_publisher_gui'
        )

def generateRobotStateController(pkg_share):
    """
    Starts by prepping the xacro file, then creating the
    robot state publisher node.
    """

    urdf_file = os.path.join(pkg_share, "urdf", "controller_qube.urdf.xacro")
    print(urdf_file)
    qube_controller = xacro.process_file(urdf_file).toxml()

    return Node(
            package='robot_state_publisher',
            executable='robot_state_publisher',
            name='robot_state_publisher',
            output='screen',
            parameters=[{
                'robot_description': qube_controller,
                'use_sim_time': True
            }]
        )

def generateQubeController():
    # Creates the qube_controller node (PID).

    return Node(
        package="qube_controller",
        executable="velocity_controller",
        name="qube_controller"
    )

def generate_launch_description():
    """
    Generates the launch description.

    Starts by getting the relevant file paths,
    then calling different functions to get the nodes.
    """
    

    python_launch_share = get_package_share_directory('qube_driver')
    rviz_share = get_package_share_directory("qube_bringup")
    robot_state_share = get_package_share_directory("qube_bringup")

    return LaunchDescription([
        generatePythonLaunch(python_launch_share),
        generateRViz(rviz_share),
        generateRobotStateController(robot_state_share),
        generateQubeController()
    ])