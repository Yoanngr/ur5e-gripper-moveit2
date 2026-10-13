import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, GroupAction, IncludeLaunchDescription, TimerAction
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    ur_type = LaunchConfiguration("ur_type")
    robot_ip = LaunchConfiguration("robot_ip")
    launch_rviz = LaunchConfiguration("launch_rviz")

    description_share = get_package_share_directory("ur5e_gripper_description")
    moveit_share = get_package_share_directory("ur5e_gripper_moveit_config")
    driver_share = get_package_share_directory("ur_robot_driver")

    description_file = os.path.join(
        description_share, "urdf", "ur5e_with_gripper.urdf.xacro"
    )
    controllers_file = os.path.join(
        description_share, "config", "ur5e_gripper_controllers.yaml"
    )

    # 1. Driver UR (matériel simulé) avec notre robot et nos contrôleurs
    driver = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(driver_share, "launch", "ur_control.launch.py")
        ),
        launch_arguments={
            "ur_type": ur_type,
            "robot_ip": robot_ip,
            "use_mock_hardware": "true",
            "launch_rviz": "false",
            "description_file": description_file,
            "controllers_file": controllers_file,
        }.items(),
    )

    # 2. Contrôleur du gripper, lancé après le driver
    gripper_spawner = TimerAction(
        period=5.0,
        actions=[
            Node(
                package="controller_manager",
                executable="spawner",
                arguments=[
                    "gripper_controller",
                    "--controller-manager-timeout",
                    "60",
                ],
                output="screen",
            )
        ],
    )

    # 3. MoveIt + RViz
    moveit = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(moveit_share, "launch", "ur5e_gripper_moveit.launch.py")
        ),
        launch_arguments={"ur_type": ur_type, "launch_rviz": launch_rviz}.items(),
    )

    driver = GroupAction(
        actions=[
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(
                    os.path.join(driver_share, "launch", "ur_control.launch.py")
                ),
                launch_arguments={
                    "ur_type": ur_type,
                    "robot_ip": robot_ip,
                    "use_mock_hardware": "true",
                    "launch_rviz": "false",
                    "description_file": description_file,
                    "controllers_file": controllers_file,
                }.items(),
            )
        ]
    )

    return LaunchDescription(
        [
            DeclareLaunchArgument("ur_type", default_value="ur5e"),
            DeclareLaunchArgument("robot_ip", default_value="yyy.yyy.yyy.yyy"),
            DeclareLaunchArgument("launch_rviz", default_value="true"),
            driver,
            gripper_spawner,
            moveit,
        ]
    )
