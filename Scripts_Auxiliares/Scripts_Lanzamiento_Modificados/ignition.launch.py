# Copyright 2023 Clearpath Robotics, Inc.
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
#
# @author Roni Kreinin (rkreinin@clearpathrobotics.com)

import os
from pathlib import Path

from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, SetEnvironmentVariable
from launch.conditions import IfCondition, UnlessCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node

ARGUMENTS = [
    DeclareLaunchArgument('use_sim_time', default_value='true',
                          choices=['true', 'false'],
                          description='use_sim_time'),
    DeclareLaunchArgument('world', default_value='warehouse',
                          description='Ignition World'),
    DeclareLaunchArgument('model', default_value='lite',
                          choices=['standard', 'lite'],
                          description='Turtlebot4 Model'),
    DeclareLaunchArgument('gui', default_value='true',
                          choices=['true', 'false'],
                          description='Launch Ignition GUI'),
]

def generate_launch_description():

    # Directories
    pkg_turtlebot4_ignition_bringup = get_package_share_directory('turtlebot4_ignition_bringup')
    pkg_turtlebot4_ignition_gui_plugins = get_package_share_directory('turtlebot4_ignition_gui_plugins')
    pkg_turtlebot4_description = get_package_share_directory('turtlebot4_description')
    pkg_irobot_create_description = get_package_share_directory('irobot_create_description')
    pkg_irobot_create_ignition_bringup = get_package_share_directory('irobot_create_ignition_bringup')
    pkg_irobot_create_ignition_plugins = get_package_share_directory('irobot_create_ignition_plugins')
    pkg_ros_ign_gazebo = get_package_share_directory('ros_ign_gazebo')

    # Set Ignition resource paths
    ign_resource_path = SetEnvironmentVariable(
        name='IGN_GAZEBO_RESOURCE_PATH',
        value=':'.join([
            os.path.join(pkg_turtlebot4_ignition_bringup, 'worlds'),
            os.path.join(pkg_irobot_create_ignition_bringup, 'worlds'),
            str(Path(pkg_turtlebot4_description).parent.resolve()),
            str(Path(pkg_irobot_create_description).parent.resolve())
        ])
    )

    ign_gui_plugin_path = SetEnvironmentVariable(
        name='IGN_GUI_PLUGIN_PATH',
        value=':'.join([
            os.path.join(pkg_turtlebot4_ignition_gui_plugins, 'lib'),
            os.path.join(pkg_irobot_create_ignition_plugins, 'lib')
        ])
    )

    # Paths
    ign_gazebo_launch = PathJoinSubstitution([pkg_ros_ign_gazebo, 'launch', 'ign_gazebo.launch.py'])

    # Ignition with GUI
    ignition_gazebo_gui = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([ign_gazebo_launch]),
        condition=IfCondition(LaunchConfiguration('gui')),
        launch_arguments=[
            ('ign_args', [
                LaunchConfiguration('world'),
                '.sdf',
                ' -r',
                ' -v 4',
                ' --gui-config ',
                PathJoinSubstitution([
                    pkg_turtlebot4_ignition_bringup,
                    'gui',
                    LaunchConfiguration('model'),
                    'gui.config'
                ])
            ])
        ]
    )

    # Ignition without GUI (-s)
    ignition_gazebo_headless = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([ign_gazebo_launch]),
        condition=UnlessCondition(LaunchConfiguration('gui')),
        launch_arguments=[
            ('ign_args', [
                LaunchConfiguration('world'),
                '.sdf',
                ' -r',
                ' -v 4',
                ' -s'
            ])
        ]
    )

    # Clock bridge
    clock_bridge = Node(
        package='ros_gz_bridge',
        executable='parameter_bridge',
        name='clock_bridge',
        output='screen',
        arguments=['/clock@rosgraph_msgs/msg/Clock[ignition.msgs.Clock']
    )

    # Create launch description and add actions
    ld = LaunchDescription(ARGUMENTS)
    ld.add_action(ign_resource_path)
    ld.add_action(ign_gui_plugin_path)
    ld.add_action(ignition_gazebo_gui)
    ld.add_action(ignition_gazebo_headless)
    ld.add_action(clock_bridge)
    return ld
