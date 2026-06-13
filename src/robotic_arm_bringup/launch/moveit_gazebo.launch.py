from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, TimerAction
from launch.launch_description_sources import PythonLaunchDescriptionSource

from launch_ros.substitutions import FindPackageShare
from launch.substitutions import PathJoinSubstitution


def generate_launch_description():

    gazebo_launch_path = PathJoinSubstitution([
        FindPackageShare('robotic_arm_gazebo'),
        'launch',
        'gazebo.launch.py'
    ])

    moveit_move_group_launch_path = PathJoinSubstitution([
        FindPackageShare('robotic_arm_moveit_config'),
        'launch',
        'move_group.launch.py'
    ])
    moveit_rviz_launch_path = PathJoinSubstitution([
        FindPackageShare('robotic_arm_moveit_config'),
        'launch',
        'moveit_rviz.launch.py'
    ])

    gazebo_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(gazebo_launch_path)
    )

    moveit_move_group_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(moveit_move_group_launch_path)
    )
    moveit_rviz_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(moveit_rviz_launch_path)
    )

    return LaunchDescription([
        gazebo_launch,

        TimerAction(
            period=5.0,
            actions=[ moveit_move_group_launch, moveit_rviz_launch ]
        )
    ])