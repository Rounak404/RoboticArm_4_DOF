from launch import LaunchDescription
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
from launch_ros.substitutions import FindPackageShare
from launch.substitutions import PathJoinSubstitution
from launch.actions import SetEnvironmentVariable, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
import os

def generate_launch_description():
    # Gazebo package
    ros_gz_sim_pkg_path = get_package_share_directory('ros_gz_sim')

    # Package paths
    robotic_arm_pkg_path = FindPackageShare('robotic_arm_gazebo')  # Replace with your own package name
    robotic_arm_desc_path = get_package_share_directory('robotic_arm_desc')  # Replace with your own description package name 

     # Gazebo launch file
    gz_launch_path = PathJoinSubstitution([ros_gz_sim_pkg_path, 'launch', 'gz_sim.launch.py'])

    # World file
    gz_world_path = PathJoinSubstitution([robotic_arm_pkg_path, 'worlds', 'robotic_arm_world.sdf'])  # Replace with your own world file

    controller_config = os.path.join(
        robotic_arm_desc_path,
        'config',
        'controllers.yaml'
    )
    print("WORLD FILE:", gz_world_path)

    urdf_file = os.path.join(
    robotic_arm_desc_path,
    'urdf',
    'robotic_arm.urdf'
)

    with open(urdf_file, 'r') as f:
        robot_description = f.read()


    robot_state_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        parameters=[
            {'robot_description': robot_description},
            {'use_sim_time': True}
        ],
        output='screen'
    )

    return LaunchDescription([
        SetEnvironmentVariable(
            'GZ_SIM_RESOURCE_PATH',
            os.path.dirname(robotic_arm_desc_path),
        ),

        # Gazebo launch file with world
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(gz_launch_path),
            launch_arguments={
                'gz_args': ['', gz_world_path],
                'on_exit_shutdown': 'true'
                }.items(),
        ),
        
        # Bridge for Gazebo clock
        Node(
            package='ros_gz_bridge',
            executable='parameter_bridge',
            arguments=[
                '/clock@rosgraph_msgs/msg/Clock[gz.msgs.Clock',
            ],
            output='screen'
        ),

        # Include the display.launch.py from the robotic_arm_desc package
        robot_state_publisher,

        Node(
            package = 'ros_gz_sim',
            executable = 'create',
            arguments = [
                '-topic', '/robot_description',
                '-name', 'robotic_arm', 
                '-x', '0.0',
                '-y', '0.0',
                '-z', '1.02'  
            ],
            output = 'screen'
        ),
        Node(
            package='controller_manager',
            executable='spawner',
            arguments=['joint_state_broadcaster']
        ),

        Node(
            package='controller_manager',
            executable='spawner',
            arguments=['arm_controller']
        ),

        Node(
            package='controller_manager',
            executable='spawner',
            arguments=['gripper_controller']
        ),
    ])