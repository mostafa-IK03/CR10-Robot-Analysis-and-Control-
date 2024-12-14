#!/usr/bin/env python3

import rclpy
from rclpy.node import Node

from sensor_msgs.msg import JointState
import numpy as np
import kinpy as kp

from custom_interfaces.msg import JointInterface


class MyNode(Node):

    def __init__(self):
        super().__init__("ForwardKinematics")

        # Declare and retrieve the URDF_PATH parameter
        self.declare_parameter("urdf_path", "/home/ralph/ros2_ws/EECE_661/src/cr10_description/urdf/CR10URDF.urdf")
        urdf_path = self.get_parameter("urdf_path").get_parameter_value().string_value

        try:
            with open(urdf_path, "rb") as f:
                urdf_text = f.read()
            self.chain_ = kp.build_serial_chain_from_urdf(urdf_text, "Link6")
        except FileNotFoundError:
            self.get_logger().error(f"URDF file not found at path: {urdf_path}")
            raise

        # Subscriptions, publications, and timers
        self.subscription_ = self.create_subscription(JointState, '/joint_states', self.listener_callback, 10)
        self.publisher_ = self.create_publisher(JointInterface, '/end_effector_pos', 10)
        self.timer_ = self.create_timer(0.1, self.publish_end_effector_pos)  # Publish at 10 Hz

        # Message to store the pose of the end effector
        self.msg_pose_ = JointInterface()

    def listener_callback(self, msg):
        # Extract joint positions
        joint1 = msg.position[0]
        joint2 = msg.position[1]
        joint3 = msg.position[2]
        joint4 = msg.position[3]
        joint5 = msg.position[4]
        joint6 = msg.position[5]

        # Compute forward kinematics
        dh = [0.0 + joint1, np.pi / 2.0 + joint2, 0.0 + joint3, -np.pi / 2.0 + joint4, 0.0 + joint5, 0.0 + joint6]
        fk_result = self.chain_.forward_kinematics(dh, end_only=False)

        # Extract end effector position and orientation
        end_effector_name = "Link6"
        end_effector_pose = fk_result[end_effector_name]

        self.msg_pose_.x_position = end_effector_pose.pos[0]
        self.msg_pose_.y_position = end_effector_pose.pos[1]
        self.msg_pose_.z_position = end_effector_pose.pos[2]
        self.msg_pose_.x_rotation = end_effector_pose.rot[0]
        self.msg_pose_.y_rotation = end_effector_pose.rot[1]
        self.msg_pose_.z_rotation = end_effector_pose.rot[2]
        self.msg_pose_.w_rotation = end_effector_pose.rot[3]

    def publish_end_effector_pos(self):
        # Publish the calculated end-effector position and orientation
        self.publisher_.publish(self.msg_pose_)


def main():
    rclpy.init()
    node = MyNode()
    rclpy.spin(node)
    rclpy.shutdown()


if __name__ == "__main__":
    main()
