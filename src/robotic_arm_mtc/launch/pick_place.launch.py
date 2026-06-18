from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, TimerAction
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node
from moveit_configs_utils import MoveItConfigsBuilder
from ament_index_python.packages import get_package_share_directory
import os

def generate_launch_description():
    # 1. Load MoveIt Config safely
    moveit_config = (
        MoveItConfigsBuilder("robotic_arm_desc", package_name="robotic_arm_moveit_config")
        .to_moveit_configs()
    )

    # 2. Paths to configuration files
    mtc_params = os.path.join(
        get_package_share_directory("robotic_arm_mtc"), "config", "robotic_arm_config.yaml"
    )
    rviz_config = os.path.join(
        get_package_share_directory("robotic_arm_moveit_config"), "config", "moveit.rviz"
    )
    controllers_yaml = os.path.join(
        get_package_share_directory("robotic_arm_desc"), "config", "controllers.yaml"
    )

    # 3. MoveGroup node
    move_group_node = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(get_package_share_directory("robotic_arm_moveit_config"), "launch", "move_group.launch.py")
        ),
        launch_arguments={"capabilities": "move_group/ExecuteTaskSolutionCapability"}.items()
    )

    # 4. RViz node (Using ONLY guaranteed existing attributes)
    rviz_node = Node(
        package="rviz2",
        executable="rviz2",
        arguments=["-d", rviz_config],
        parameters=[
            moveit_config.robot_description,
            moveit_config.robot_description_semantic,
            moveit_config.robot_description_kinematics,
            moveit_config.planning_pipelines,
        ],
        output="screen",
    )

    # 5. Controller Manager (The core ros2_control node)
    ros2_control_node = Node(
        package="controller_manager",
        executable="ros2_control_node",
        parameters=[
            moveit_config.robot_description,
            controllers_yaml
        ],
        output="screen",
    )

    # 6. Spawners (These MUST connect to the controller manager)
    arm_controller_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=["arm_controller", "-c", "/controller_manager"],
    )
    
    gripper_controller_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=["gripper_controller", "-c", "/controller_manager"],
    )

    # 7. MTC Demo Node
    pick_place_node = Node(
        package="robotic_arm_mtc",
        executable="pick_place_demo",
        output="screen",
        parameters=[moveit_config.to_dict(), mtc_params],
    )
    robot_state_publisher_node = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        output="both",
        parameters=[moveit_config.robot_description],
    )
    joint_state_broadcaster_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=["joint_state_broadcaster", "-c", "/controller_manager"],
    )

    return LaunchDescription([
        robot_state_publisher_node,
        move_group_node,
        rviz_node,
        ros2_control_node,
        
        # Delay the spawners by 2 seconds to ensure the controller_manager is fully up
        TimerAction(period=2.0, actions=[joint_state_broadcaster_spawner]),
        TimerAction(period=2.0, actions=[arm_controller_spawner]),
        TimerAction(period=2.0, actions=[gripper_controller_spawner]),
        
        # Delay the MTC node by 8 seconds to allow the scene and controllers to initialize
        TimerAction(period=8.0, actions=[pick_place_node])
    ])