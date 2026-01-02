from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, RegisterEventHandler
from launch.conditions import IfCondition
from launch.event_handlers import OnProcessExit
from launch.substitutions import Command, FindExecutable, PathJoinSubstitution, LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    # Declare arguments
    declared_arguments = []
    declared_arguments.append(
        DeclareLaunchArgument(
            "use_sim_time",
            default_value="true",
            description="Use simulation (Gazebo) clock if true",
        )
    )
    declared_arguments.append(
        DeclareLaunchArgument(
            "description_package",
            default_value="rrbot_3dof_description",
            description="Description package with robot URDF/xacro file",
        )
    )
    declared_arguments.append(
        DeclareLaunchArgument(
            "description_file",
            default_value="rrbot_3dof.urdf.xacro",
            description="URDF file name",
        )
    )
    declared_arguments.append(
        DeclareLaunchArgument(
            "controllers_file",
            default_value="rrbot_controllers.yaml",
            description="YAML file with controllers configuration",
        )
    )

    # Initialize Arguments
    use_sim_time = LaunchConfiguration("use_sim_time")
    description_package = LaunchConfiguration("description_package")
    description_file = LaunchConfiguration("description_file")
    controllers_file = LaunchConfiguration("controllers_file")

    # Get URDF via xacro (but we use plain URDF, so just read it)
    robot_description_content = Command(
        [
            PathJoinSubstitution([FindExecutable(name="xacro")]),
            " ",
            PathJoinSubstitution(
                [FindPackageShare(description_package), "urdf", description_file]
            ),
        ]
    )
    # If you're using plain URDF (not xacro), replace the above with:
    # robot_description_content = ParameterValue(
    #     PathJoinSubstitution([FindPackageShare(description_package), "urdf", description_file]),
    #     value_type=str
    # )

    robot_description = {"robot_description": robot_description_content}

    # Get controller config path
    robot_controllers = PathJoinSubstitution(
        [FindPackageShare(description_package), "config", controllers_file]
    )

    # Nodes
    control_node = Node(
        package="controller_manager",
        executable="ros2_control_node",
        parameters=[robot_controllers, robot_description],
        output="both",
        condition=IfCondition(use_sim_time),  # Only in sim
    )

    robot_state_pub_node = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        output="both",
        parameters=[robot_description, {"use_sim_time": use_sim_time}],
    )

    joint_state_broadcaster_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=["joint_state_broadcaster", "--controller-manager", "/controller_manager"],
    )

    robot_controller_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=["position_trajectory_controller", "--controller-manager", "/controller_manager"],
    )

    rviz_config = PathJoinSubstitution(
        [FindPackageShare(description_package), "config", "rrbot.rviz"]
    )
    rviz_node = Node(
        package="rviz2",
        executable="rviz2",
        arguments=["-d", rviz_config],
    )

    # Delay start of robot controller to after joint_state_broadcaster
    delay_robot_controller_spawner_after_joint_state_broadcaster = RegisterEventHandler(
        event_handler=OnProcessExit(
            target_action=joint_state_broadcaster_spawner,
            on_exit=[robot_controller_spawner],
        )
    )

    nodes = [
        rviz_node,
        control_node,
        robot_state_pub_node,
        joint_state_broadcaster_spawner,
        delay_robot_controller_spawner_after_joint_state_broadcaster,
    ]

    return LaunchDescription(declared_arguments + nodes)