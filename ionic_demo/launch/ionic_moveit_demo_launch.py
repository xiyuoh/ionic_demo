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

from ament_index_python.packages import get_package_share_directory
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare

from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import Command, FindExecutable, PathJoinSubstitution, TextSubstitution


def generate_launch_description():
    ionic_demo_dir = Path(get_package_share_directory('ionic_demo'))
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
                'panda_y': '0.2',
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
                "-y", "0.2",
                "-z", "1.2",
            ],
            parameters=[{"use_sim_time": True}],
        ),
        Node(
            # Spawn a can for delivery
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
        )
    ])

