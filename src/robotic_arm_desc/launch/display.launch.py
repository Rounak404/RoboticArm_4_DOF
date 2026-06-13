import launch
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
import os
import xacro

def generate_launch_description():

    pkg_name = 'robotic_arm_desc'
    pkg_path = FindPackageShare(pkg_name).find(pkg_name)

    xacro_file = os.path.join(
        pkg_path,
        'urdf',
        'robotic_arm.urdf.xacro'
    )

    robot_description_config = xacro.process_file(xacro_file)

    robot_description = robot_description_config.toxml()
    


    return launch.LaunchDescription([

        Node(
            package='robot_state_publisher',
            executable='robot_state_publisher',
            name='robot_state_publisher',
            parameters=[
                {'robot_description': robot_description},
            ],
            output='screen'
        ),

        Node(
            package='joint_state_publisher_gui',
            executable='joint_state_publisher_gui',
            name='joint_state_publisher_gui',
            output='screen'
        ),
        Node(
            package='rviz2',
            executable='rviz2',
            output='screen'
        )
    ])