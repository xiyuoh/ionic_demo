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

class RobotAPIResult(enum.IntEnum):
    SUCCESS = 0
    """The request was successful"""

    RETRY = 1
    """The client failed to connect but might succeed if you try again"""

    IMPOSSIBLE = 2
    """The client connected but something about the request is impossible"""


class RobotAPI:
    # The constructor below accepts parameters typically required to submit
    # http requests. Users should modify the constructor as per the
    # requirements of their robot's API
    def __init__(self, prefix: str, user: str, password: str):
        self.prefix = prefix
        self.user = user
        self.password = password
        self.timeout = 5.0
        self.debug = False

        

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
        ########################################################################
        # IMPLEMENT CODE HERE
        ########################################################################
        return None


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
