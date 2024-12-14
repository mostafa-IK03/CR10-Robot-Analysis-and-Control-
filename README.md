# CR10 Robot Analysis and Control

This repository contains code and resources for analyzing and controlling a CR10 collaborative robot as part of the EECE 661 project at the American University of Beirut, Maroun Semaan Faculty of Engineering and Architecture, Department of Electrical and Computer Engineering.

**Project Report:**  
A detailed project report is available in this repository. It includes analyses of Forward and Inverse Kinematics, Jacobian computation, URDF modeling, visualization in Rviz2, and trajectory planning (both linear interpolation and using Ruckig).

**Video Demonstrations:**  
Several steps of the project are demonstrated through videos, including robot visualization in Rviz2, forward kinematics computation, and trajectory execution. Links are embedded in the report.

## Features

- **Forward Kinematics (FK)**: Compute the end-effector position and orientation given the robot’s joint parameters using the [Kinpy library](https://github.com/neka-nat/kinpy).
- **Inverse Kinematics (IK)**: Determine the joint angles required to achieve a desired end-effector pose.
- **Jacobian Matrix Computation**: Compute the Jacobian matrix to map joint velocities to end-effector velocities.
- **URDF Modeling and Rviz2 Visualization**: Visualize the CR10 robot model in [Rviz2](https://docs.ros.org/en/humble/index.html) and interactively manipulate joints.
- **Trajectory Planning**:
  - Method 1: Linear interpolation between points.
  - Method 2: Smooth, jerk-limited trajectories using the [Ruckig](https://pypi.org/project/ruckig) library.
- **Parametric Paths**: Generate complex parametric trajectories (e.g., circles) and visualize them in Rviz2.

## Getting Started

### Prerequisites

- **ROS2 (Humble or above)**: [Installation Instructions](https://docs.ros.org/en/humble/index.html)
- **Python Packages**:
  - [Kinpy](https://github.com/neka-nat/kinpy)
  - [Ruckig](https://pypi.org/project/ruckig)
- **CR10 URDF Files**: Adapted from the official CR10 URDF repository by Dobot.  
  CR10 Info: [https://www.dobot-robots.com/products/cr-series/cr10.html](https://www.dobot-robots.com/products/cr-series/cr10.html) 
  Joint Limits and Specs: [https://unchainedrobotics.de/en/products/robot/cobot/dobot-cr10](https://unchainedrobotics.de/en/products/robot/cobot/dobot-cr10) 

### Installation

1. **Clone the Repository**:
   ```bash
   git clone https://github.com/mostafa-IK03/CR10-Robot-Analysis-and-Control-.git
   cd CR10-Robot-Analysis-and-Control-
