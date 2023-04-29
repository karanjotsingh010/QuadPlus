#!/usr/bin/env python

"""
Example to launch a sensor_combined listener node.

.. seealso::
    https://index.ros.org/doc/ros2/Launch-system/
"""

from launch import LaunchDescription
import launch_ros.actions
import os

def generate_launch_description():
    if os.environ['ROS_DISTRO'] != "galactic" and os.environ['ROS_DISTRO'] != "rolling":
        return LaunchDescription([
           launch_ros.actions.Node(
                package='px4_ros_com', node_executable='sensor_integrator_INDI', output='screen'),
           launch_ros.actions.Node(
                package='px4_ros_com', node_executable='CA_INDI_POS', output='screen'),
           launch_ros.actions.Node(
                package='px4_ros_com', node_executable='servo_publisher', output='screen'),
                
        ])
    else:
        return LaunchDescription([
            launch_ros.actions.Node(
                package='px4_ros_com', executable='sensor_integrator_INDI', output='screen'),
            launch_ros.actions.Node(
                package='px4_ros_com', executable='CA_INDI_POS', output='screen'),
            launch_ros.actions.Node(
                package='px4_ros_com', executable='servo_publisher', output='screen'),
        ])
