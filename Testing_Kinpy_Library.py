import numpy as np
import kinpy as kp

# Open the URDF file and read its content
with open("CR10URDF.urdf", "rb") as f:
    urdf_text = f.read()

# Build the serial chain using the URDF file content
chain = kp.build_serial_chain_from_urdf(urdf_text, "Link6")
print(chain)
print("Joint Parameter Names:", chain.get_joint_parameter_names())

# Define joint angles (example configuration)
th = [0.0, np.pi / 2.0, 0.0, -np.pi / 2.0, 0.0, 0.0]

# Compute forward kinematics
fk_result = chain.forward_kinematics(th, end_only=False)
print("\nForward Kinematics Result:")
print(fk_result)

# Extract the end-effector pose (position and orientation)
end_effector_name = "Link6"
end_effector_pose = fk_result[end_effector_name]

print("\nEnd-Effector Position:", end_effector_pose.pos)
print("End-Effector Orientation (Quaternion):", end_effector_pose.rot)

# -------------------------------
# Inverse Kinematics Calculation
# -------------------------------

# Define the desired end-effector pose (same as the one obtained from FK for testing)
desired_pose = kp.Transform(rot=end_effector_pose.rot, pos=end_effector_pose.pos)

# Compute inverse kinematics
ik_solution = chain.inverse_kinematics(desired_pose)

if ik_solution is not None:
    print("\nInverse Kinematics Solution (Joint Angles):")
    for name, angle in zip(chain.get_joint_parameter_names(), ik_solution):
        print(f"{name}: {np.degrees(angle):.2f} degrees")
else:
    print("\nInverse kinematics did not converge to a solution.")

# -------------------------------
# Jacobian Calculation
# -------------------------------

# Compute the Jacobian matrix at the current joint configuration
jacobian_matrix = chain.jacobian(th)

print("\nJacobian Matrix:")
print(jacobian_matrix)
