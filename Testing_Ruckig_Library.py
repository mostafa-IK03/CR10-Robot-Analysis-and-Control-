import ruckig
import matplotlib.pyplot as plt

# Define the number of Degrees of Freedom (DoFs) and control cycle time
otg = ruckig.Ruckig(1, 0.01)  # 1 DoF, 10 ms control cycle
# Input parameters: current state, target state, and kinematic limits
input_param = ruckig.InputParameter(1)
input_param.current_position = [0.0]
input_param.current_velocity = [0.0]
input_param.current_acceleration = [0.0]
input_param.target_position = [1.0]
input_param.target_velocity = [0.0]
input_param.target_acceleration = [0.0]
input_param.max_velocity = [2.0]
input_param.max_acceleration = [2.0]
input_param.max_jerk = [10.0]
# Output parameters
output_param = ruckig.OutputParameter(1)
# Lists to store trajectory data for plotting
time_data = []
position_data = []
velocity_data = []
acceleration_data = []
# Generate the trajectory
print("Generating trajectory...")
while otg.update(input_param, output_param) == ruckig.Result.Working:
    time_data.append(output_param.time)
    position_data.append(output_param.new_position[0])
    velocity_data.append(output_param.new_velocity[0])
    acceleration_data.append(output_param.new_acceleration[0])
    output_param.pass_to_input(input_param)
print("Trajectory complete!")
# Plot the trajectory
plt.figure(figsize=(10, 6))
# Plot Position
plt.subplot(3, 1, 1)
plt.plot(time_data, position_data, label="Position")
plt.ylabel("Position (m)")
plt.title("Ruckig Trajectory")
plt.grid(True)
plt.legend()
# Plot Velocity
plt.subplot(3, 1, 2)
plt.plot(time_data, velocity_data, label="Velocity", color="orange")
plt.ylabel("Velocity (m/s)")
plt.grid(True)
plt.legend()
# Plot Acceleration
plt.subplot(3, 1, 3)
plt.plot(time_data, acceleration_data, label="Acceleration", color="green")
plt.xlabel("Time (s)")
plt.ylabel("Acceleration (m/s²)")
plt.grid(True)
plt.legend()
plt.tight_layout()
plt.show()

