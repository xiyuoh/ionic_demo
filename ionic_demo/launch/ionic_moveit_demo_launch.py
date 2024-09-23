# Copyright (C) 2024 Open Source Robotics Foundation
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#       http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.

from pathlib import Path

from ament_index_python.packages import get_package_share_directory, get_package_prefix
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare, FindPackagePrefix
from launch_xml.launch_description_sources import XMLLaunchDescriptionSource

from launch import LaunchDescription
from launch.actions import AppendEnvironmentVariable, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import Command, FindExecutable, PathJoinSubstitution, TextSubstitution


def generate_launch_description():
    ionic_demo_dir = Path(get_package_share_directory('ionic_demo'))
    rmf_demo_assets_dir = Path(get_package_share_directory('rmf_demos_assets')) / 'models'
    rmf_plugins_dir = Path(get_package_prefix('rmf_robot_sim_gz_plugins')) / 'lib' / 'rmf_robot_sim_gz_plugins'
    rmf_demos_dir = Path(get_package_share_directory('rmf_demos'))
    ionic_maps_dir = Path(get_package_share_directory('ionic_demo_building_maps'))
    fleet_adapter_dir = Path(get_package_share_directory('fleet_adapter_nav2'))
    coke_can_string = \
    '''
    <sdf version="1.6">
        <include>
            <static>false</static>
            <uri>
                https://fuel.gazebosim.org/1.0/OpenRobotics/models/Coke
            </uri>
        </include>
    </sdf>
    '''

    teleport_ingestor_string = \
    '''
    <sdf version="1.6">
        <include>
            <name>coke_ingestor</name>
            <uri>
                model://TeleportIngestor
            </uri>
        </include>
    </sdf>
    '''

    set_resource_path_vars = AppendEnvironmentVariable(
        'GZ_SIM_RESOURCE_PATH', str(rmf_demo_assets_dir)
    )
    set_plugin_path_vars = AppendEnvironmentVariable(
        'GZ_SIM_SYSTEM_PLUGIN_PATH', str(rmf_plugins_dir)
    )

    # URDF
    _robot_description_xml = Command(
        [
            PathJoinSubstitution([FindExecutable(name="xacro")]),
            " ",
            PathJoinSubstitution(
                [FindPackageShare('panda_description'), 'urdf', 'panda.urdf.xacro']
            ),
            " ",
            "name:=",
            'panda',
        ]
    )
    robot_description = {"robot_description": _robot_description_xml}

    # SRDF
    _robot_description_semantic_xml = Command(
        [
            PathJoinSubstitution([FindExecutable(name="xacro")]),
            " ",
            PathJoinSubstitution(
                [
                    FindPackageShare('panda_moveit_config'),
                    "srdf",
                    "panda.srdf.xacro",
                ]
            ),
            " ",
            "name:=",
            'panda',
        ]
    )
    robot_description_semantic = {
        "robot_description_semantic": _robot_description_semantic_xml
    }
    tb4_namespace = '/tb4'
    nav2_bringup_dir = Path(get_package_share_directory('nav2_bringup'))
    nav2_launch_dir = nav2_bringup_dir / 'launch'

    return LaunchDescription([
        set_resource_path_vars,
        set_plugin_path_vars,
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource([
                PathJoinSubstitution([
                    FindPackageShare('panda'),
                    'launch',
                    'gz.launch.py'
                ])
            ]),
            launch_arguments={
                'world': PathJoinSubstitution([
                   FindPackageShare('ionic_demo'),
                      'worlds',
                      'ionic.sdf'
                ]),
                'panda_x': '4.3',
                'panda_y': '0.3',
                'panda_z': '1.2',
            }.items()
        ),
        Node(
            # Spawn a can for delivery
            package="ros_gz_sim",
            executable="create",
            output="log",
            arguments=[
                "-string", coke_can_string,
                "-x", "4.9",
                "-y", "0.3",
                "-z", "1.2",
            ],
            parameters=[{"use_sim_time": True}],
        ),
        Node(
            # Spawn a can for delivery
            package="ros_gz_sim",
            executable="create",
            output="log",
            arguments=[
                "-string", teleport_ingestor_string,
                "-x", "-0.6",
                "-y", "2.6",
                "-z", "0.9",
            ],
            parameters=[{"use_sim_time": True}],
        ),
        Node(
            # Dispenser node to control moveit arm
            package="ionic_demos_rmf",
            executable="moveit_dispenser",
            output="log",
            parameters=[
                robot_description,
                robot_description_semantic,
                {"use_sim_time": True},
            ],
        ),
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                str(ionic_demo_dir / 'launch' / 'tb4_spawn_launch.py')
            ),
        ),
        # Launch RMF and RMF fleet adapter
        IncludeLaunchDescription(
            XMLLaunchDescriptionSource(
                str(rmf_demos_dir / 'common.launch.xml')
            ),
            launch_arguments={
                'use_sim_time': 'True',
                'headless': 'True',
                'config_file': str(ionic_maps_dir / 'ionic_demo' / 'ionic_demo.building.yaml'),
            }.items(),
        ),
        Node(
            package="fleet_adapter_nav2",
            executable="fleet_adapter",
            output="log",
            arguments=[
                "-c", str(fleet_adapter_dir / 'config' / 'tb4_config.yaml'),
                "-n", str(ionic_maps_dir / 'maps' / 'ionic_demo' / 'nav_graphs' / '0.yaml')
            ],
            parameters=[{"use_sim_time": True}],
            remappings=[
                ('/tf', '/tb4/tf'),
                ('/tf_static', '/tb4/tf_static'),
            ]
        ),
    ])

