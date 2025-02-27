from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch_ros.actions import Node


def generate_launch_description():
    use_sim_time_arg = DeclareLaunchArgument(
        "use_sim_time",
        default_value="True",
    )
    wheel_radius_arg = DeclareLaunchArgument(
        "wheel_radius",
        default_value="0.0325",
    )
    wheel_separation_arg = DeclareLaunchArgument(
        "wheel_separation",
        default_value="0.1488",
    )
    wheel_radius_error_arg = DeclareLaunchArgument(
        "wheel_radius_error",
        default_value="0.005",
    )
    wheel_separation_error_arg = DeclareLaunchArgument(
        "wheel_separation_error",
        default_value="0.01",
    )

    joint_state_broadcaster_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=[
            "joint_state_broadcaster",
            "--controller-manager",
            "/controller_manager",
        ],
    )

    # Comentamos el spawner del diff_drive_controller
    # wheel_controller_spawner = Node(
    #     package="controller_manager",
    #     executable="spawner",
    #     arguments=["twip_controller",
    #         "--controller-manager",
    #         "/controller_manager"
    #     ],
    # )

    effort_controller_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=["twip_effort_controller",
                   "--controller-manager",
                   "/controller_manager"
                   ],
    )

    return LaunchDescription(
        [
            use_sim_time_arg,
            wheel_radius_arg,
            wheel_separation_arg,
            wheel_radius_error_arg,
            wheel_separation_error_arg,
            joint_state_broadcaster_spawner,
            # wheel_controller_spawner,  # Comentado
            effort_controller_spawner,
        ]
    )
