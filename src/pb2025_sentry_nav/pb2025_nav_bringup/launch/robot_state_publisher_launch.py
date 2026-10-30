# Copyright 2025 Lihan Chen
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.


# NOTE: This startup file is only used when the navigation module is standalone
# It is used to launch the robot state publisher and joint state publisher.
# But in a complete robot system, this part should be completed by an independent robot startup module

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, GroupAction, IncludeLaunchDescription
from launch.conditions import LaunchConfigurationEquals, LaunchConfigurationNotEquals
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import PushRosNamespace, SetRemap
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    # Get the launch directory
    legacy_description_launch = PathJoinSubstitution([
        FindPackageShare("pb2025_robot_description"),
        "launch", "robot_description_launch.py",
    ])

    namespace = LaunchConfiguration("namespace")
    use_sim_time = LaunchConfiguration("use_sim_time")
    robot_name = LaunchConfiguration("robot_name")

    # Declare the launch arguments
    declare_namespace_cmd = DeclareLaunchArgument(
        "namespace",
        default_value="",
        description="Top-level namespace",
    )

    declare_use_sim_time_cmd = DeclareLaunchArgument(
        "use_sim_time",
        default_value="false",
        description="Use simulation (Gazebo) clock if true",
    )

    declare_robot_name_cmd = DeclareLaunchArgument(
        "robot_name",
        # default_value="simulation_robot",
        default_value="lc2025_sentry_robot",
        description="The file name of the robot xmacro to be used",
    )

    bringup_cmd_group = GroupAction(
        [
            PushRosNamespace(namespace=namespace),
            SetRemap("/tf", "tf"),
            SetRemap("/tf_static", "tf_static"),
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(
                    legacy_description_launch
                ),
                condition=LaunchConfigurationNotEquals("robot_name", "lc_sentry2027"),
                launch_arguments={
                    "namespace": namespace,
                    "use_sim_time": use_sim_time,
                    "robot_name": robot_name,
                    "use_rviz": "False",
                }.items(),
            ),
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(PathJoinSubstitution([
                    FindPackageShare("lc_robot_description"),
                    "launch", "lc_sentry2027.launch.py",
                ])),
                condition=LaunchConfigurationEquals("robot_name", "lc_sentry2027"),
                launch_arguments={
                    "namespace": "",
                    "use_sim_time": use_sim_time,
                    "use_rviz": "False",
                    "joint_state_source": LaunchConfiguration("joint_state_source"),
                    "use_joint_state_publisher": LaunchConfiguration("use_joint_state_publisher"),
                }.items(),
            ),
        ]
    )

    # Create the launch description and populate
    ld = LaunchDescription()

    # Declare the launch options
    ld.add_action(declare_namespace_cmd)
    ld.add_action(declare_use_sim_time_cmd)
    ld.add_action(declare_robot_name_cmd)
    ld.add_action(DeclareLaunchArgument("joint_state_source", default_value="serial/gimbal_joint_state"))
    ld.add_action(DeclareLaunchArgument("use_joint_state_publisher", default_value="true"))

    # Add the actions to launch all nodes
    ld.add_action(bringup_cmd_group)

    return ld
