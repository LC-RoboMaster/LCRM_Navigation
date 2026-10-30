import os
from pathlib import Path

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, GroupAction
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node, PushRosNamespace, SetRemap
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description():
    share = get_package_share_directory("lc_robot_description")
    robot_description = Path(share, "urdf", "lc_sentry2027.urdf").read_text(encoding="utf-8")
    use_sim_time = LaunchConfiguration("use_sim_time")
    return LaunchDescription([
        DeclareLaunchArgument("namespace", default_value=""),
        DeclareLaunchArgument("use_sim_time", default_value="false"),
        DeclareLaunchArgument("use_rviz", default_value="true"),
        DeclareLaunchArgument("use_joint_state_publisher", default_value="true"),
        DeclareLaunchArgument(
            "joint_state_source", default_value="serial/gimbal_joint_state",
            description="JointState input using the joint names in lc_sentry2027.urdf",
        ),
        GroupAction([
            PushRosNamespace(LaunchConfiguration("namespace")),
            SetRemap("/tf", "tf"),
            SetRemap("/tf_static", "tf_static"),
            Node(
                package="robot_state_publisher", executable="robot_state_publisher",
                name="robot_state_publisher", output="screen",
                parameters=[{
                    "use_sim_time": use_sim_time,
                    "robot_description": ParameterValue(robot_description, value_type=str),
                }],
            ),
            Node(
                condition=IfCondition(LaunchConfiguration("use_joint_state_publisher")),
                package="joint_state_publisher", executable="joint_state_publisher",
                name="joint_state_publisher", output="screen",
                parameters=[{
                    "use_sim_time": use_sim_time,
                    "rate": 50,
                    "source_list": [LaunchConfiguration("joint_state_source")],
                }],
            ),
            Node(
                condition=IfCondition(LaunchConfiguration("use_rviz")),
                package="rviz2", executable="rviz2", output="screen",
                arguments=["-d", os.path.join(share, "rviz", "lc_sentry2027.rviz")],
                parameters=[{"use_sim_time": use_sim_time}],
            ),
        ]),
    ])
