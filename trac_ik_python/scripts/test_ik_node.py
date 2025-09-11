#!/usr/bin/env python3
import math

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Pose
from sensor_msgs.msg import JointState

import numpy as np

from trac_ik_python import trac_ik_python_module


class IKNode(Node):
    def __init__(self):
        super().__init__('test_ik_node')

        self.declare_parameter('robot_description', '')
        self.declare_parameter('base_link', '')
        self.declare_parameter('tip_link', '')
        self.declare_parameter('joint_names', [''])

        self.__robot_description: str = self.get_parameter('robot_description').get_parameter_value().string_value
        self.__base_link : str = self.get_parameter('base_link').get_parameter_value().string_value
        self.__tip_link : int = self.get_parameter('tip_link').get_parameter_value().string_value
        self.__joint_names : list = self.get_parameter('joint_names').get_parameter_value().string_array_value

        self.__joint_state_publisher = self.create_publisher(JointState, "/joint_states", 10)
        self.__solver_subscription =  self.create_subscription(Pose, "/solve_ik", self.__solve_ik_subscription, 10)

        try:
            self.trac_ik = trac_ik_python_module.TrackIKBindings(
                "test_ik_node", self.__robot_description, self.__base_link, self.__tip_link)
            self.get_logger().info("trac-ik C++ wrapper initialized")
        except Exception as e:
            self.get_logger().error(f"failed to init trac-ik wrapper: {e}")

        self.__current_joint_state  = np.zeros(len(self.__joint_names), dtype=float)
        self.__current_joint_state[0] = math.radians(74.01)
        self.__current_joint_state[1] = math.radians(-80.44)
        self.__current_joint_state[2] = math.radians(64.59)
        self.__current_joint_state[3] = math.radians(-98.68)
        self.__current_joint_state[4] = math.radians(-90.89)
        self.__current_joint_state[5] = math.radians(-15.93)

        lower_boundary = np.array([-0.2, -3.14, -3.14, -3.14, -3.14, -3.14])
        upper_boundary = np.array([3.87, 3.14, 3.14, 3.14, 3.14, 3.14])
        self.trac_ik.set_joint_limits(lower_boundary, upper_boundary)

        self.create_timer(0.1, self.__tick)

        self.__solver_subscription


    def __solve_ik_subscription(self, msg : Pose):
        cartesian_position = np.array([
            msg.position.x,
            msg.position.y,
            msg.position.z,
            msg.orientation.x,
            msg.orientation.y,
            msg.orientation.z,
            msg.orientation.w], dtype=float)
        self.get_logger().info("computing inverse kinematics")
        self.__current_joint_state = \
            self.trac_ik.perform_inverse_kinematics_trac_ik(self.__current_joint_state, cartesian_position)


    def __tick(self):
        self.get_logger().info("joint states")
        self.get_logger().info("j1: " + str(math.degrees(self.__current_joint_state[0])))
        self.get_logger().info("j2: " + str(math.degrees(self.__current_joint_state[1])))
        self.get_logger().info("j3: " + str(math.degrees(self.__current_joint_state[2])))
        self.get_logger().info("j4: " + str(math.degrees(self.__current_joint_state[3])))
        self.get_logger().info("j5: " + str(math.degrees(self.__current_joint_state[4])))
        self.get_logger().info("j6: " + str(math.degrees(self.__current_joint_state[5])))
        
        self.get_logger().info("computing forward kinematics")
        result = self.trac_ik.perform_forward_kinematics(self.__current_joint_state)
        self.get_logger().info("cartesian pose")
        self.get_logger().info("x: " + str(result[0]))
        self.get_logger().info("y: " + str(result[1]))
        self.get_logger().info("z: " + str(result[2]))
        self.get_logger().info("rx: " + str(result[3]))
        self.get_logger().info("ry: " + str(result[4]))
        self.get_logger().info("rz: " + str(result[5]))
        self.get_logger().info("rw: " + str(result[6]))

        joint_state = JointState()
        joint_state.name = self.__joint_names
        joint_state.header.stamp = self.get_clock().now().to_msg()
        joint_state.position.append(self.__current_joint_state[0])
        joint_state.position.append(self.__current_joint_state[1])
        joint_state.position.append(self.__current_joint_state[2])
        joint_state.position.append(self.__current_joint_state[3])
        joint_state.position.append(self.__current_joint_state[4])
        joint_state.position.append(self.__current_joint_state[5])
        joint_state.velocity = [0.0, 0.0, 0.0, 0.0, 0.0, 0.0]
        joint_state.effort = [0.0, 0.0, 0.0, 0.0, 0.0, 0.0]

        self.__joint_state_publisher.publish(joint_state)


def main():
    rclpy.init()
    node = IKNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()