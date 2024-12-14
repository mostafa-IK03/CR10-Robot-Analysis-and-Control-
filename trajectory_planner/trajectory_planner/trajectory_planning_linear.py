#!/usr/bin/env python3

import rclpy
import numpy as np
from rclpy.node import Node
import kinpy as kp
import math
from visualization_msgs.msg import Marker
from custom_interfaces.msg import EndEffectorCommand
from sensor_msgs.msg import JointState
from custom_interfaces.msg import JointInterface
from geometry_msgs.msg import Point


END_EFFECTOR_LINK = "Link6"

class MyNode(Node):

    def __init__(self):
        super().__init__("Planner")

        # Declare and retrieve the URDF_PATH parameter
        self.declare_parameter("urdf_path", "/home/ralph/ros2_ws/EECE_661/src/cr10_description/urdf/CR10URDF.urdf")
        urdf_path = self.get_parameter("urdf_path").get_parameter_value().string_value

        try:
            with open(urdf_path, "rb") as f:
                urdf_text = f.read()
            self.chain = kp.build_serial_chain_from_urdf(urdf_text, END_EFFECTOR_LINK)
        except FileNotFoundError:
            self.get_logger().error(f"URDF file not found at path: {urdf_path}")
            raise

        self.end_effector_pos = [0] * 7
        self.joint_trajectory = []  

        self.subscriber_ = self.create_subscription(EndEffectorCommand, "/desired_pos", self.desired_position_callback, 10)
        self.subscriber_pose = self.create_subscription(JointInterface, "/end_effector_pos", self.update_position, 10)
        self.publisher_ = self.create_publisher(JointState, "/joint_states", 10)
        self.marker_publisher_ = self.create_publisher(Marker, "/trajectory_marker", 10)

        # Timer callback for periodic joint state publishing
        self.timer = self.create_timer(0.1, self.publish_joint_states)  # Publish at 10 Hz

    def desired_position_callback(self, msg):
        point_a = msg.point_a
        point_b = msg.point_b
        linear_speed = msg.linear_speed

        # Get the inverse kinematics of the start point and end point
        pose_at_a = kp.Transform(rot=[0, 0, 0, 1], pos=[point_a.x, point_a.y, point_a.z])
        ika_solution = self.chain.inverse_kinematics(pose_at_a)

        pose_at_b = kp.Transform(rot=[0, 0, 0, 1], pos=[point_b.x, point_b.y, point_b.z])
        ikb_solution = self.chain.inverse_kinematics(pose_at_b)

        # Check that both the start and end point are reachable in the task space 
        if ika_solution is not None and ikb_solution is not None:
            print("\nCalculated Inverse Kinematics")
        else:
            print("\nInverse kinematics did not converge to a solution.")

        # Find the distance between the two points
        distance = math.sqrt(
            (point_b.x - point_a.x) ** 2 +
            (point_b.y - point_a.y) ** 2 +
            (point_b.z - point_a.z) ** 2
        )

        if linear_speed <= 0:
            self.get_logger().error("Linear speed must be positive.")
            return

        # Compute the time for the path
        duration = distance / linear_speed
        steps = max(1, int(duration * 10))  #  at least one step

        # Calculate each subpoint path
        delta = [
            (point_b.x - point_a.x) / steps,
            (point_b.y - point_a.y) / steps,
            (point_b.z - point_a.z) / steps
        ]

        self.joint_trajectory = []  

        # Find each "subpoint" in the path, find its inverse kinematics, and add it to a list
        points = []
        for i in range(steps):
            intermediate_position = kp.Transform(rot=[0, 0, 0, 1], pos=[
                point_a.x + delta[0] * i,
                point_a.y + delta[1] * i,
                point_a.z + delta[2] * i
            ])

            joint_angles = self.chain.inverse_kinematics(intermediate_position)
            if joint_angles is not None:
                self.joint_trajectory.append(joint_angles)

                # Add trajectory point for RViz visualization
                points.append((
                    point_a.x + delta[0] * i,
                    point_a.y + delta[1] * i,
                    point_a.z + delta[2] * i
                ))
            else:
                self.get_logger().warn("Failed to compute IK for step {}".format(i))

        self.publish_trajectory_marker(points)
        self.get_logger().info("Generated joint trajectory with {} steps".format(len(self.joint_trajectory)))

    def publish_joint_states(self):
        if not self.joint_trajectory:
            return  # No trajectory to publish

        msg_joint = JointState()
        msg_joint.header.stamp = self.get_clock().now().to_msg()
        msg_joint.name = ['joint1', 'joint2', 'joint3', 'joint4', 'joint5', 'joint6']

        # Publish the next step in the trajectory
        next_angles = self.joint_trajectory.pop(0)  # Remove the first element
        msg_joint.position = [float(angle) for angle in next_angles]

        self.publisher_.publish(msg_joint)
        self.get_logger().info(f'Published joint state: {msg_joint.position}')

    def publish_trajectory_marker(self, points):
        marker = Marker()
        marker.header.frame_id = "base_link"
        marker.header.stamp = self.get_clock().now().to_msg()
        marker.ns = "trajectory"
        marker.id = 0
        marker.type = Marker.LINE_STRIP
        marker.action = Marker.ADD

        marker.scale.x = 0.02  # Line width

        marker.color.r = 1.0
        marker.color.g = 0.0
        marker.color.b = 0.0
        marker.color.a = 1.0  # Fully opaque

        # Add points to the marker
        for point in points:
            p = Point()
            p.x, p.y, p.z = point
            marker.points.append(p)

        self.marker_publisher_.publish(marker)
        self.get_logger().info("Published trajectory marker")

    def update_position(self, msg):
        self.end_effector_pos[0] = msg.x_position
        self.end_effector_pos[1] = msg.y_position
        self.end_effector_pos[2] = msg.z_position
        self.end_effector_pos[3] = msg.x_rotation
        self.end_effector_pos[4] = msg.y_rotation
        self.end_effector_pos[5] = msg.z_rotation
        self.end_effector_pos[6] = msg.w_rotation


def main():
    rclpy.init()
    node = MyNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
