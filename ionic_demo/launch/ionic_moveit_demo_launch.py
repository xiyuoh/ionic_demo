from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare

from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import Command, FindExecutable, PathJoinSubstitution, TextSubstitution


def generate_launch_description():
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
            PythonLaunchDescriptionSource([
                PathJoinSubstitution([
                    FindPackageShare('nav2_bringup'),
                    'launch',
                    'tb4_simulation_launch.py'
                ])
            ]),
            launch_arguments={
                'map': PathJoinSubstitution([
                   FindPackageShare('ionic_demo'),
                      'maps',
                      'ionic_demo.yaml'
                ]),
                'world': PathJoinSubstitution([
                   FindPackageShare('ionic_demo'),
                      'worlds',
                      'ionic.sdf'
                ]),
                'namespace': '/tb4',
                'use_namespace': 'true',
                'headless': '0',
                'use_simulator': 'False',
                'x_pose': '0.0',
                'y_pose': '0.0',
                'z_pose': '0.0'
            }.items()
        )
    ])

