import os
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, TimerAction, SetEnvironmentVariable
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node
from moveit_configs_utils import MoveItConfigsBuilder
from ament_index_python.packages import get_package_share_directory

def generate_launch_description():
    # 1. Package Shared Directory Paths
    bringup_pkg_share = get_package_share_directory("robotic_arm_bringup")
    gazebo_pkg_share = get_package_share_directory("robotic_arm_gazebo")
    moveit_pkg_share = get_package_share_directory("robotic_arm_moveit_config")
    mtc_pkg_share = get_package_share_directory("robotic_arm_mtc")
    desc_pkg_share = get_package_share_directory("robotic_arm_desc")

    # 2. CRITICAL: Load MoveIt Config passing the 'use_gazebo' mapping argument
    moveit_config = (
        MoveItConfigsBuilder("robotic_arm_desc", package_name="robotic_arm_moveit_config")
        .robot_description(mappings={'use_gazebo': 'true'}) # Maps matching your xacro target block
        .to_moveit_configs()
    )

    # 3. Path Variables
    mtc_params = os.path.join(mtc_pkg_share, "config", "robotic_arm_config.yaml")
    rviz_config = os.path.join(moveit_pkg_share, "config", "moveit.rviz")
    gz_world_path = os.path.join(gazebo_pkg_share, "worlds", "robotic_arm_world.sdf")
    gz_launch_path = os.path.join(get_package_share_directory("ros_gz_sim"), "launch", "gz_sim.launch.py")

    # =========================================================================
    # CORE SIMULATION NODES (From gazebo.launch.py context)
    # =========================================================================
    
    # Run the underlying Ignition Gazebo simulation framework server
    gazebo_sim = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(gz_launch_path),
        launch_arguments={
            'gz_args': f'-r -s {gz_world_path}',
            'on_exit_shutdown': 'true'
        }.items(),
    )

    # Clock synchronization bridge to handle /use_sim_time transitions
    parameter_bridge = Node(
        package='ros_gz_bridge',
        executable='parameter_bridge',
        arguments=['/clock@rosgraph_msgs/msg/Clock[gz.msgs.Clock'],
        output='screen'
    )

    # Spawns physical URDF representation into the active Gazebo instance
    spawn_robot_entity = Node(
        package='ros_gz_sim',
        executable='create',
        arguments=[
            '-topic', '/robot_description',
            '-name', 'robotic_arm', 
            '-x', '0.0', '-y', '0.0', '-z', '1.02'  
        ],
        output='screen'
    )

    # Global robot state publisher tracking tf transforms matching sim environment clock
    robot_state_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        parameters=[
            moveit_config.robot_description, # Cleanly pulls the mapped gazebo description
            {'use_sim_time': True}
        ],
        output='screen'
    )

    # =========================================================================
    # ROS2 CONTROL TRAJECTORY CONTROLLERS (Executed sequentially)
    # =========================================================================
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

    # =========================================================================
    # HIGH-LEVEL PLANNING INTERFACES (From pick_place.launch.py context)
    # =========================================================================
    
    # Launch MoveGroup action handlers mapping Capability task solvers
    move_group_node = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(moveit_pkg_share, "launch", "move_group.launch.py")),
        launch_arguments={"capabilities": "move_group/ExecuteTaskSolutionCapability"}.items()
    )

    # Visual monitoring dashboard tracking simulated layout constraints
    rviz_node = Node(
        package="rviz2",
        executable="rviz2",
        arguments=["-d", rviz_config],
        parameters=[
            moveit_config.robot_description,
            moveit_config.robot_description_semantic,
            moveit_config.robot_description_kinematics,
            moveit_config.planning_pipelines,
            {'use_sim_time': True}
        ],
        output="screen",
    )

    # The C++ MTC automated sequence pipeline node
    pick_place_node = Node(
        package="robotic_arm_mtc",
        executable="pick_place_demo",
        output="screen",
        parameters=[moveit_config.to_dict(), mtc_params, {'use_sim_time': True}],
    )

    # =========================================================================
    # COORDINATED SCHEDULING TIME SEQUENCES
    # =========================================================================
    
    # Controller triggers wait until Gazebo engine spawns entities completely
    delayed_broadcaster = TimerAction(period=3.0, actions=[joint_state_broadcaster])
    delayed_arm_control = TimerAction(period=6.0, actions=[arm_controller])
    delayed_grip_control = TimerAction(period=9.0, actions=[gripper_controller])

    # MoveIt planning layers boot after control buses stabilize
    delayed_moveit_stack = TimerAction(period=10.0, actions=[move_group_node, rviz_node])

    # MTC executes once entire environment loop stabilizes
    delayed_mtc_pipeline = TimerAction(period=15.0, actions=[pick_place_node])

    return LaunchDescription([
        # Set asset location resources map
        SetEnvironmentVariable('GZ_SIM_RESOURCE_PATH', os.path.dirname(desc_pkg_share)),
        
        # Base Simulation Environment
        gazebo_sim,
        parameter_bridge,
        robot_state_publisher,
        spawn_robot_entity,

        # Coordinated sequence cascades
        delayed_broadcaster,
        delayed_arm_control,
        delayed_grip_control,
        delayed_moveit_stack,
        delayed_mtc_pipeline
    ])