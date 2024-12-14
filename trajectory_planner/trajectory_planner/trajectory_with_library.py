#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
import kinpy as kp
from ruckig import Ruckig, InputParameter, OutputParameter, Result
from math import sqrt
from visualization_msgs.msg import Marker
from custom_interfaces.msg import EndEffectorCommand, ParametricCommand
from sensor_msgs.msg import JointState
from custom_interfaces.msg import JointInterface
from geometry_msgs.msg import Point
import numpy as np

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
        self.joint_trajectory = []  # Ruckig trajectory

        # Subscribers
        self.subscriber_pose = self.create_subscription(JointInterface, "/end_effector_pos", self.update_position, 10)
        self.subscriber_desired_pos = self.create_subscription(EndEffectorCommand, "/desired_pos", self.desired_position_callback, 10)
        self.subscriber_parametric = self.create_subscription(ParametricCommand, "/parametric_command", self.parametric_callback, 10)

        # Publishers
        self.publisher_ = self.create_publisher(JointState, "/joint_states", 10)
        self.marker_publisher_ = self.create_publisher(Marker, "/trajectory_marker", 10)

        # Timer callback for periodic joint state publishing
        self.timer = self.create_timer(0.1, self.publish_joint_states)  # Publish at 10 Hz

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

    def desired_position_callback(self, msg):
        point_a = msg.point_a
        point_b = msg.point_b
        linear_speed = msg.linear_speed  # Desired Cartesian linear speed

        pose_at_a = kp.Transform(rot=[0, 0, 0, 1], pos=[point_a.x, point_a.y, point_a.z])
        ika_solution = self.chain.inverse_kinematics(pose_at_a)

        pose_at_b = kp.Transform(rot=[0, 0, 0, 1], pos=[point_b.x, point_b.y, point_b.z])
        ikb_solution = self.chain.inverse_kinematics(pose_at_b)

        if ika_solution is None or ikb_solution is None:
            self.get_logger().error("Inverse kinematics did not converge.")
            return

        cartesian_distance = sqrt(
            (point_b.x - point_a.x) ** 2 +
            (point_b.y - point_a.y) ** 2 +
            (point_b.z - point_a.z) ** 2
        )

        if cartesian_distance == 0:
            self.get_logger().error("Path length is zero. No motion required.")
            return

        if linear_speed <= 0:
            self.get_logger().error("Linear speed must be positive.")
            return

        trajectory_duration = cartesian_distance / linear_speed

        dofs = len(ika_solution)
        otg = Ruckig(dofs, 0.1)  # Control cycle: 100ms
        input_param = InputParameter(dofs)
        output_param = OutputParameter(dofs)

        max_joint_velocities = [2.094, 2.094, 3.142, 3.142, 3.142, 3.142]  # rad/s
        input_param.current_position = ika_solution
        input_param.current_velocity = [0.0] * dofs
        input_param.current_acceleration = [0.0] * dofs
        input_param.target_position = ikb_solution
        input_param.target_velocity = [0.0] * dofs
        input_param.target_acceleration = [0.0] * dofs

        scaling_factor = linear_speed / cartesian_distance
        input_param.max_velocity = [min(v * scaling_factor, v) for v in max_joint_velocities]
        input_param.max_acceleration = [0.5] * dofs  # rad/s²
        input_param.max_jerk = [1.0] * dofs  # rad/s³

        self.joint_trajectory = []
        points = []

        result = otg.update(input_param, output_param)
        while result == Result.Working:
            self.joint_trajectory.append(output_param.new_position)

            input_param.current_position = output_param.new_position
            input_param.current_velocity = output_param.new_velocity
            input_param.current_acceleration = output_param.new_acceleration

            result = otg.update(input_param, output_param)

            fk_result = self.chain.forward_kinematics(output_param.new_position)
            points.append((fk_result.pos[0], fk_result.pos[1], fk_result.pos[2]))

        if result == Result.Finished:
            self.get_logger().info(f"Generated joint trajectory with {len(self.joint_trajectory)} steps using Ruckig.")
        else:
            self.get_logger().warn("Ruckig failed to complete trajectory generation.")

        self.publish_trajectory_marker(points)

    def parametric_callback(self, msg):
        center_x = msg.center.x
        center_y = msg.center.y
        center_z = msg.center.z
        radius = msg.radius
        num_waypoints = msg.num_waypoints
        linear_speed = msg.linear_speed

        MIN_SAFE_DISTANCE = 0.5
        MAX_REACH_DISTANCE = 1.2
        MIN_Z = 0.2
        MAX_Z = 1.2

        distance_from_base = sqrt(center_x**2 + center_y**2 + center_z**2)
        if distance_from_base - radius < MIN_SAFE_DISTANCE:
            self.get_logger().warn(
                f"Circle too close to the robot (distance: {distance_from_base}). Adjusting center."
            )
            scaling_factor = (radius + MIN_SAFE_DISTANCE) / distance_from_base
            center_x *= scaling_factor
            center_y *= scaling_factor
            center_z = max(center_z, MIN_Z + radius)

        if distance_from_base + radius > MAX_REACH_DISTANCE:
            self.get_logger().error(
                f"Circle is out of reach (distance: {distance_from_base + radius})."
            )
            return

        center_z = min(max(center_z, MIN_Z), MAX_Z)

        orientation_axis = np.array([center_x, center_y, center_z])
        orientation_axis = orientation_axis / np.linalg.norm(orientation_axis)
        rotation_matrix = self.calculate_rotation_matrix(orientation_axis)

        thetas = np.linspace(0, 2 * np.pi, num_waypoints, endpoint=False)
        cartesian_waypoints = []
        for theta in thetas:
            circle_point = np.array([
                radius * np.cos(theta),
                radius * np.sin(theta),
                0
            ])
            rotated_point = rotation_matrix @ circle_point
            waypoint = rotated_point + np.array([center_x, center_y, center_z])
            cartesian_waypoints.append(waypoint.tolist())

        joint_waypoints = []
        for point in cartesian_waypoints:
            pose = kp.Transform(rot=[0, 0, 0, 1], pos=point)
            ik_solution = self.chain.inverse_kinematics(pose)
            if ik_solution is None:
                self.get_logger().error("IK failed for a waypoint.")
                return
            joint_waypoints.append(ik_solution)

        self.generate_trajectory(joint_waypoints, linear_speed, cartesian_waypoints)

    def calculate_rotation_matrix(self, axis):
        z_axis = np.array([0, 0, 1])
        axis = axis / np.linalg.norm(axis)
        v = np.cross(z_axis, axis)
        c = np.dot(z_axis, axis)
        s = np.linalg.norm(v)

        if s == 0:
            return np.eye(3)

        v_skew = np.array([
            [0, -v[2], v[1]],
            [v[2], 0, -v[0]],
            [-v[1], v[0], 0]
        ])
        rotation_matrix = np.eye(3) + v_skew + v_skew @ v_skew * ((1 - c) / (s**2))
        return rotation_matrix

    def generate_trajectory(self, joint_waypoints, linear_speed, cartesian_waypoints):
        dofs = len(joint_waypoints[0])
        otg = Ruckig(dofs, 0.1)
        input_param = InputParameter(dofs)
        output_param = OutputParameter(dofs)

        max_joint_velocities = [2.094] * dofs
        input_param.max_velocity = max_joint_velocities
        input_param.max_acceleration = [0.5] * dofs
        input_param.max_jerk = [1.0] * dofs

        current_q = joint_waypoints[0]
        input_param.current_position = current_q
        input_param.current_velocity = [0.0] * dofs
        input_param.current_acceleration = [0.0] * dofs

        trajectory_points = []

        for target_q in joint_waypoints[1:]:
            input_param.target_position = target_q
            input_param.target_velocity = [0.0] * dofs
            input_param.target_acceleration = [0.0] * dofs

            result = otg.update(input_param, output_param)
            if result not in [Result.Working, Result.Finished]:
                self.get_logger().error("Ruckig failed to generate trajectory.")
                return

            trajectory_points.append(output_param.new_position)

            input_param.current_position = output_param.new_position
            input_param.current_velocity = output_param.new_velocity
            input_param.current_acceleration = output_param.new_acceleration

        self.joint_trajectory = trajectory_points
        self.get_logger().info(f"Generated circular trajectory with {len(trajectory_points)} points.")
        self.publish_trajectory_marker(cartesian_waypoints)

    def publish_trajectory_marker(self, points):
        marker = Marker()
        marker.header.frame_id = "base_link"
        marker.header.stamp = self.get_clock().now().to_msg()
        marker.ns = "trajectory"
        marker.id = 0
        marker.type = Marker.LINE_STRIP
        marker.action = Marker.ADD

        marker.scale.x = 0.02

        marker.color.r = 1.0
        marker.color.g = 0.0
        marker.color.b = 0.0
        marker.color.a = 1.0

        for point in points:
            p = Point()
            p.x, p.y, p.z = point
            marker.points.append(p)

        self.marker_publisher_.publish(marker)
        self.get_logger().info("Published trajectory marker")

    def update_position(self, msg):
        self.end_effector_pos[0] = float(msg.x_position)
        self.end_effector_pos[1] = float(msg.y_position)
        self.end_effector_pos[2] = float(msg.z_position)
        self.end_effector_pos[3] = float(msg.x_rotation)
        self.end_effector_pos[4] = float(msg.y_rotation)
        self.end_effector_pos[5] = float(msg.z_rotation)
        self.end_effector_pos[6] = float(msg.w_rotation)


def main():
    rclpy.init()
    node = MyNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
