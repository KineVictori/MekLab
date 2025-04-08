from launch import LaunchDescription
from launch_ros.actions import Node
from launch.substitutions import Command, FindExecutable, PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare

# generate_launch_description er 'main' funksjonen i launch fila.
# Den returnerer et LaunchDescription-objekt som inneholder alle nodene som skal startes
def generate_launch_description():

    # pkg_name er navnet til pakka
    pkg_name = 'qube_description'
    # pkg_share er pathen til share mappa til pakka
    pkg_share = FindPackageShare(package=pkg_name).find(pkg_name)

    # default_rviz_config_path er pathen til rviz config fila
    default_rviz_config_path = PathJoinSubstitution([pkg_share, 'rviz/qube_config.rviz'])

    # robot_state_publisher_node er brukt til å publishe robotens tilstand
    robot_state_publisher_node = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        name='robot_state_publisher',
        output='screen',
        parameters=[{
            'robot_description': Command([
                FindExecutable(name='xacro'), ' ',
                PathJoinSubstitution([pkg_share, 'urdf/qube.urdf.xacro'])
            ])
        }]
    )

    # joint_state_publisher_node er brukt til å publisere joint tilstander
    joint_state_publisher_node = Node(
        package='joint_state_publisher_gui',
        executable='joint_state_publisher_gui',
        name='joint_state_publisher_gui'
    )

    # rviz_node blir brukt for å visualisere roboten
    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        output='screen',
        arguments=['-d', default_rviz_config_path],
    )

    # returner launch description
    return LaunchDescription([
        robot_state_publisher_node,
        rviz_node,
        joint_state_publisher_node
    ])
