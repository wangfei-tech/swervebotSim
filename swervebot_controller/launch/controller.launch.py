from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, GroupAction, OpaqueFunction
from launch.actions import RegisterEventHandler
from launch.event_handlers import OnProcessExit
from launch_ros.actions import Node


def generate_launch_description():
    
    use_sim_time_arg = DeclareLaunchArgument(
        "use_sim_time",
        default_value="True",
    )

    joint_state_broadcaster_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=[
            "joint_state_broadcaster",
            "--controller-manager",
            "/controller_manager",
        ]
    )

    caster_controller_spawner = Node(
        package='controller_manager',
        executable='spawner',
        arguments=[
            'caster_trajectory_controller',
            '--controller-manager', 
            '/controller_manager',
        ],
    )

    wheel_controller_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=[
            "wheel_trajectory_controller",
            "--controller-manager",
            "/controller_manager",
        ],
    )

    swervebot_controller = Node(
        package="swervebot_controller",
        executable="swerve_controller",
    )

    scan_merger = Node(
        package="scan_merger",
        executable="scan_merger",
        parameters=[
            {'scan1_topic': '/backright/scan'},
            {'scan2_topic': '/topleft/scan'},
            {'output_topic': '/scan'},
            {'angle_min': -3.14159265},
            {'angle_max': 3.14159265},
            {'angle_increment': 0.004363323}
        ]
    )

    delay_caster_controller_spawner_after_joint_state_broadcaster_spawner = RegisterEventHandler(
        event_handler=OnProcessExit(
            target_action=joint_state_broadcaster_spawner,
            on_exit=[caster_controller_spawner],
        )
    )

    delay_wheel_controller_spawner_after_caster_controller_spawner = RegisterEventHandler(
        event_handler=OnProcessExit(
            target_action=caster_controller_spawner,
            on_exit=[wheel_controller_spawner],
        )
    )


    return LaunchDescription(
        [
            use_sim_time_arg,
            joint_state_broadcaster_spawner,
            delay_caster_controller_spawner_after_joint_state_broadcaster_spawner,
            delay_wheel_controller_spawner_after_caster_controller_spawner,
            swervebot_controller,
            scan_merger
        ]
    )