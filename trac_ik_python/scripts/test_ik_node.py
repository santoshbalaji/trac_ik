#!/usr/bin/env python3

import math
import time

import rclpy
from rclpy.node import Node

import numpy as np

from trac_ik_python import trac_ik_python_module

class IKNode(Node):
    def __init__(self):
        super().__init__('test_ik_node')

        # Declare parameters with defaults (can be overridden by launch file)
        self.declare_parameter('robot_description', '')

        # Get parameter values
        self.robot_description: str = self.get_parameter('robot_description').get_parameter_value().string_value

        # Basic sanity checks
        if not self.robot_description:
            self.get_logger().error("robot_description parameter is empty. Did the launch file pass the xacro?")
        else:
            self.get_logger().info(f"Received robot_description of length {len(self.robot_description)}")
        try:
            self.tracik = trac_ik_python_module.TrackIKBindings(
                "test_ik_node", self.robot_description, "base", "tool0")
            self.get_logger().info("TRAC-IK C++ wrapper initialized.")
        except Exception as e:
            self.get_logger().error(f"Failed to init TRAC-IK wrapper: {e}")

        self.create_timer(1.0, self._tick)

    def _tick(self):
        self.get_logger().info("tick executed")
        current_position = np.array([math.radians(202.39), 0.0, 0.0, 0.0, 0.0, 0.0], dtype=float)
        cartesian_position = np.array([0.74, 0.47, 0.45, 0.5899, -0.5629, 0.41636, -0.4019], dtype=float)

        self.get_logger().info("computing inverse kinematics")
        start_time = time.perf_counter()
        result = self.tracik.perform_inverse_kinematics_trac_ik(current_position, cartesian_position)
        end_time = time.perf_counter()
        total_time = end_time - start_time
        self.get_logger().info("total time: " + str(total_time))
        self.get_logger().info("j1: " + str(math.degrees(result[0])))
        self.get_logger().info("j2: " + str(math.degrees(result[1])))
        self.get_logger().info("j3: " + str(math.degrees(result[2])))
        self.get_logger().info("j4: " + str(math.degrees(result[3])))
        self.get_logger().info("j5: " + str(math.degrees(result[4])))
        self.get_logger().info("j6: " + str(math.degrees(result[5])))
        
        self.get_logger().info("computing forward kinematics")
        current_position = np.array(
            [math.radians(202.39), 
             math.radians(-54.10), 
             math.radians(107.39), 
             math.radians(-212.62),
             math.radians(-110.76), 
             math.radians(5.27)], dtype=float)
        result = self.tracik.perform_forward_kinematics(current_position)
        self.get_logger().info("x: " + str(result[0]))
        self.get_logger().info("y: " + str(result[1]))
        self.get_logger().info("z: " + str(result[2]))
        self.get_logger().info("rx: " + str(result[3]))
        self.get_logger().info("ry: " + str(result[4]))
        self.get_logger().info("rz: " + str(result[5]))
        self.get_logger().info("rw: " + str(result[6]))


def main():
    rclpy.init()
    node = IKNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()