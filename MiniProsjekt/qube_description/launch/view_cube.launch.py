from launch import LaunchDescription
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
import os

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='robot_state_publisher',
            executable='robot_state_publisher',
            name='robot_state_publisher',
            output='screen',
            parameters=[{
                'robot_description': 
                    Command([
                        'xacro ',
                        os.path.join(
                            get_package_share_directory('qube_description'),
                            'urdf/qube.urdf.xacro'
                        )
                    ])
            }]
        ),
        Node(
            package='rviz2',
            executable='rviz2',
            name='rviz2',
            output='screen',
            arguments=['-d', os.path.join(
                get_package_share_directory('qube_description'),
                'rviz/view_qube.rviz'
            )]
        )
    ])
