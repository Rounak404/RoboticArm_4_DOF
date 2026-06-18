from launch import LaunchDescription
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
from launch_ros.substitutions import FindPackageShare
from launch.substitutions import PathJoinSubstitution
from launch.actions import SetEnvironmentVariable, IncludeLaunchDescription, TimerAction
from launch.launch_description_sources import PythonLaunchDescriptionSource
import xacro
import os

def generate_launch_description():
    # Gazebo package
    ros_gz_sim_pkg_path = get_package_share_directory('ros_gz_sim')

    # Package paths
    robotic_arm_pkg_path = FindPackageShare('robotic_arm_gazebo')  
    robotic_arm_desc_path = get_package_share_directory('robotic_arm_desc')  
    
    # 🟢 Calculate the master share directory correctly
    workspace_share_dir = os.path.dirname(robotic_arm_desc_path)

    # Gazebo launch file
    gz_launch_path = PathJoinSubstitution([ros_gz_sim_pkg_path, 'launch', 'gz_sim.launch.py'])

    # World file
    gz_world_path = PathJoinSubstitution([robotic_arm_pkg_path, 'worlds', 'robotic_arm_world.sdf'])  

    controller_config = os.path.join(
        robotic_arm_desc_path,
        'config',
        'controllers.yaml'
    )

    xacro_file = os.path.join(
        robotic_arm_desc_path,
        'urdf',
        'robotic_arm.urdf.xacro'
    )

    robot_description_config = xacro.process_file(xacro_file, mappings={'use_gazebo': 'true'})
    robot_description = robot_description_config.toxml()

    robot_state_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        parameters=[
            {'robot_description': robot_description},
            {'use_sim_time': True}
        ],
        output='screen'
    )

    joint_state_broadcaster = Node(
        package='controller_manager',
        executable='spawner',
        arguments=['joint_state_broadcaster'],
        output='screen'
    )

    arm_controller = Node(
        package='controller_manager',
        executable='spawner',
        arguments=['arm_controller'],
        output='screen'
    )

    gripper_controller = Node(
        package='controller_manager',
        executable='spawner',
        arguments=['gripper_controller'],
        output='screen'
    )

    return LaunchDescription([
        # 🟢 FIXED: Map BOTH environment keys uniformly to workspace_share_dir
        SetEnvironmentVariable(
            name='GZ_SIM_RESOURCE_PATH',
            value=workspace_share_dir,
        ),
        SetEnvironmentVariable(
            name='GAZEBO_MODEL_PATH',
            value=workspace_share_dir
        ),

        # Gazebo launch file with world
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(gz_launch_path),
            launch_arguments={
                'gz_args': ['-r ','-s ', gz_world_path],
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
        joint_state_broadcaster,

        TimerAction(
            period=5.0,
            actions=[arm_controller]
        ),

        TimerAction(
            period=8.0,
            actions=[gripper_controller]
        ),
    ])