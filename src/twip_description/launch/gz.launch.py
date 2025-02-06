# This file launches gazebo classical (not Ignition) and spawns the robot.
import os
from pathlib import Path
from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, SetEnvironmentVariable
from launch.substitutions import Command, LaunchConfiguration
from launch.launch_description_sources import PythonLaunchDescriptionSource

from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description():
    twip_description = get_package_share_directory("twip_description")

    model_arg = DeclareLaunchArgument(
        name="model",
        default_value=os.path.join(
            twip_description, "urdf", "twip.urdf.xacro"),
        description="Absolute path to robot urdf file"
    )

    # Establece la variable GAZEBO_MODEL_PATH
    gazebo_resource_path = SetEnvironmentVariable(
        name="GAZEBO_MODEL_PATH",
        value=[str(Path(twip_description).parent.resolve())]
    )

    # Genera la descripción del robot usando Xacro
    robot_description = ParameterValue(
        Command(["xacro ", LaunchConfiguration("model")]),
        value_type=str
    )

    # Publica la descripción del robot
    robot_state_publisher_node = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        parameters=[
            {"robot_description": robot_description, "use_sim_time": True}]
    )

    # Inicia Gazebo clásico (no Ignition)
    gazebo = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            os.path.join(get_package_share_directory(
                "gazebo_ros"), "launch", "gazebo.launch.py")
        ])
    )

    # Lanza el modelo dentro de Gazebo
    spawn_entity = Node(
        package="gazebo_ros",
        executable="spawn_entity.py",
        arguments=["-entity", "twip", "-file", LaunchConfiguration("model")],
        output="screen"
    )

    return LaunchDescription([
        model_arg,
        gazebo_resource_path,
        robot_state_publisher_node,
        gazebo,
        spawn_entity
    ])
