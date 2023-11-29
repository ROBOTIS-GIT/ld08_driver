#!/usr/bin/env python3
#
# Copyright 2021 ROBOTIS CO., LTD.
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
# Authors: Will Son


from launch import LaunchDescription
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    frame_id = LaunchConfiguration("frame_id", default="base_scan")
    namespace = LaunchConfiguration('namespace')

    return LaunchDescription(
        [
            Node(
                namespace=namespace,
                package="ld08_driver",
                executable="ld08_driver",
                name="ld08_driver",
                parameters=[{"frame_id": frame_id}],
                output="screen",
            ),
        ]
    )
