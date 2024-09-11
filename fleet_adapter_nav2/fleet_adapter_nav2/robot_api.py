# Copyright 2024 Open Source Robotics Foundation, Inc.
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

import enum

import rclpy
from tf2_ros import TransformException
from tf2_ros.buffer import Buffer as TransformBuffer
import tf_transformations


class RobotAPIResult(enum.IntEnum):
    SUCCESS = 0
    """The request was successful"""

    RETRY = 1
    """The client failed to connect but might succeed if you try again"""

    IMPOSSIBLE = 2
    """The client connected but something about the request is impossible"""


class RobotUpdateData:
    """Update data for a single robot."""

    def __init__(self, data):
        self.robot_name = data['robot_name']
        position = data['position']
        x = position['x']
        y = position['y']
        yaw = position['yaw']
        self.position = [x, y, yaw]
        self.map = data['map_name']
        self.battery_soc = data['battery'] / 100.0
        self.requires_replan = data.get('replan', False)

        # TODO(@xiyuoh) check task completion using nav2 API inside get_data instead
        self.last_request_completed = data['last_completed_request']

    def is_command_completed(self, cmd_id):
        return self.last_request_completed == cmd_id


class RobotAPI:
    # The constructor below accepts parameters typically required to submit
    # http requests. Users should modify the constructor as per the
    # requirements of their robot's API
    def __init__(self, prefix: str, tf_buffer: TransformBuffer):
        self.prefix = prefix
        self.base_frame = f"{self.prefix}/base_footprint"
        self.map_frame = f"{self.prefix}/map"
        self.tf_buffer = tf_buffer
        self.timeout = 5.0
        self.debug = False

        # TODO(AC): Populate these fields properly via ROS 2 subscriptions and
        # nav2 simple commander API feedback
        self.battery_soc = 100 - 1e-9
        self.last_completed_request = None


    def check_connection(self):
        """Return True if connection to the robot API server is successful."""
        ########################################################################
        # IMPLEMENT CODE HERE
        ########################################################################
        return False

    def navigate(
        self,
        robot_name: str,
        cmd_id: int,
        pose,
        map_name: str,
        speed_limit=0.0,
    ):
        """
        Request the robot to navigate to pose:[x,y,theta].

        Where x, y and theta are in the robot's coordinate convention.
        This function should return True if the robot has accepted the request,
        else False.
        """
        assert len(pose) > 2
        ########################################################################
        # IMPLEMENT CODE HERE
        ########################################################################
        return False

    def start_activity(
        self, robot_name: str, cmd_id: int, activity: str, label: str
    ):
        """
        Request the robot to begin a process.

        This is specific to the robot and the use case.
        """
        ########################################################################
        # IMPLEMENT CODE HERE
        ########################################################################
        return RobotAPIResult.RETRY

    def stop(self, robot_name: str, cmd_id: int):
        """
        Command the robot to stop.

        Return True if robot has successfully stopped. Else False
        """
        ########################################################################
        # IMPLEMENT CODE HERE
        ########################################################################
        return False

    def get_data(self, robot_name: str | None = None):
        """
        Return a RobotUpdateData for one robot if a name is given.

        Otherwise return a list of RobotUpdateData for all robots.
        """
        if robot_name is not self.robot_name:
            raise RuntimeError(
                f"Calling get_data with robot [{robot_name}] while RobotAPI is for robot [{self.robot_name}]"
            )

        transform_stamped = None
        try:
            transform_stamped = self.tf_buffer.lookup_transform(
                self.base_frame,
                self.map_frame,
                rclpy.time.Time())
        except TransformException as ex:
            self.get_logger().info(
                f'Could not transform {self.base_frame} to {self.map_frame}: {ex}')
            return None

        theta = tf_transformations.euler_from_quaternion([
            transform_stamped.transform.rotation.x,
            transform_stamped.transform.rotation.y,
            transform_stamped.transform.rotation.z,
            transform_stamped.transform.rotation.w
        ])[2]
        position = {
            "x": transform_stamped.transform.translation.x,
            "y": transform_stamped.transform.translation.y,
            "yaw": theta,
        }
        data = {
            "robot_name": self.robot_name,
            "position": position,
            "map_name": self.map_name,
            "battery": self.battery_soc,
            "last_completed_request": self.last_completed_request
        }
        return data
