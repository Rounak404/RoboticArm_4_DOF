import launch
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
import os
def generate_launch_description():

    pkg_name = 'robotic_arm_desc'
    pkg_path = FindPackageShare(pkg_name).find(pkg_name)

    urdf_file = os.path.join(pkg_path, 'urdf', 'robotic_arm.urdf')
    controller_file = os.path.join(pkg_path, 'config', 'controllers.yaml')

    with open(urdf_file, 'r') as infp:
        robot_description = infp.read()

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
            package='controller_manager',
            executable='ros2_control_node',
            parameters=[
                controller_file
            ],
            remappings = [
                ('robot_description', '/robot_description')
            ],
            output='screen'
        ),
        Node(
            package='controller_manager',
            executable='spawner',
            arguments=["joint_state_broadcaster"]
        ),
        Node(
            package='controller_manager',
            executable='spawner',
            arguments=["arm_controller"]
        ),
        Node(
            package='controller_manager',
            executable='spawner',
            arguments=["gripper_controller"]
        ),

        Node(
            package='rviz2',
            executable='rviz2',
            output='screen'
        )
    ])