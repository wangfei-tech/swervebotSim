from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, GroupAction, OpaqueFunction
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

    wheel_controller_spawner = Node(
        package='controller_manager',
        executable='spawner',
        arguments=[
            'caster_trajectory_controller',
            '--controller-manager', 
            '/controller_manager',
        ],
    )

    lift_controller_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=[
            "wheel_trajectory_controller",
            "--controller-manager",
            "/controller_manager",
        ],
    )


    return LaunchDescription(
        [
            use_sim_time_arg,
            joint_state_broadcaster_spawner,
            wheel_controller_spawner,
            lift_controller_spawner,
        ]
    )