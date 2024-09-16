import os
import sys
import time
import unittest
import uuid

import launch
from launch.launch_service import LaunchService
import launch_ros
import launch_ros.actions
import launch_testing.actions
from launch_testing.io_handler import ActiveIoHandler
import launch_testing_ros

import os
from ament_index_python.packages import (
    get_package_share_directory,
)
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

import pytest

import rclpy
from rclpy.node import Node

import std_msgs.msg
from std_msgs.msg import String

from send_behavior_tree_client import SendBehaviorTreeTester


@pytest.mark.launch_test
def generate_test_description():

    bt_action_server_node = launch_ros.actions.Node(
        package="bt_action_server",
        executable="send_bt_action_server_node",
        name="send_bt_action_server_node",
        output="screen",
        emulate_tty=True
    )
    
    service_server = launch_ros.actions.Node(
        package="bt_demo_skills",
        executable="service_server",
        name="service_server",
        output="screen",
        emulate_tty=True
    )

    return launch.LaunchDescription(
            [
                DeclareLaunchArgument('config_file_path', 
                    default_value=os.path.join(get_package_share_directory("bt_action_server"), "config", "default.yaml"), 
                    description="Path to the node configuration file."),
                DeclareLaunchArgument('behavior_tree_definitions_path', 
                    default_value=os.path.join(get_package_share_directory("bt_action_server"), "config", "trees/behavior_trees.yaml"), 
                    description="Path to the behavior trees configuration file."),
                DeclareLaunchArgument('behavior_trees_path', 
                    default_value=os.path.join(get_package_share_directory("bt_action_server"), "config", "trees/"), 
                    description="Path to the behavior trees configuration file."),
                bt_action_server_node,
                service_server,
                # Start tests right away - no need to wait for anything
                launch_testing.actions.ReadyToTest(),
            ]
        )


class TestFixture(unittest.TestCase):
    def listener_callback(self, msg):
        self.callback = msg.data
        return

    def test_publish_string(self, proc_output):
        path = "/behavior_tree_stack_ws/src/bt_action_server/test/tree/test_tree.xml"
        node = SendBehaviorTreeTester(path)
        self.subscription = node.create_subscription(
            String,
            'test',
            self.listener_callback,
            10)
        
        while not node.is_result():
            rclpy.spin_once(node)
        
        assert node.get_result() == True
        assert self.callback == "Hello world!"


    def test_success(self, proc_output):
        path = "/behavior_tree_stack_ws/src/bt_action_server/test/tree/test_tree.xml"
        node = SendBehaviorTreeTester(path)
        while not node.is_result():
            rclpy.spin_once(node)
        
        assert node.get_result() == True

    def test_loop_success(self, proc_output):
        path = "/behavior_tree_stack_ws/src/bt_action_server/test/tree/test_tree.xml"
        for t in range(3):
            node = SendBehaviorTreeTester(path)
            while not node.is_result():
                rclpy.spin_once(node)
            
            assert node.get_result() == True

    def test_nonsense(self, proc_output):
        path = "/behavior_tree_stack_ws/src/bt_action_server/test/tree/test_tree_nonsense.xml"
        node = SendBehaviorTreeTester(path)
        while not node.is_result():
            rclpy.spin_once(node)
        
        assert node.get_result() == False

    def test_loop_nonsense(self, proc_output):
        rclpy.init()
        path = "/behavior_tree_stack_ws/src/bt_action_server/test/tree/test_tree_nonsense.xml"
        for t in range(3):
            node = SendBehaviorTreeTester(path)
            while not node.is_result():
                rclpy.spin_once(node)
            
            assert node.get_result() == False

    def test_tree_2_nonsense_on_second_execution(self, proc_output):
        path1 = "/behavior_tree_stack_ws/src/bt_action_server/test/tree/test_tree.xml"
        path2 = "/behavior_tree_stack_ws/src/bt_action_server/test/tree/test_tree_2.xml"
        node = SendBehaviorTreeTester(path1)
        while not node.is_result():
            rclpy.spin_once(node)
        
        assert node.get_result() == True

        node2 = SendBehaviorTreeTester(path2)
        while not node2.is_result():
            rclpy.spin_once(node2)
        
        assert node2.get_result() == False