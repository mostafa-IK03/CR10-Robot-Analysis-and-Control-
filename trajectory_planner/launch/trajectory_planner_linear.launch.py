import os

from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node
from launch.substitutions import LaunchConfiguration, TextSubstitution


def generate_launch_description():
    urdf_path = os.path.join(
      get_package_share_directory('trajectory_planner'),
      'urdf',
      'CR10URDF.urdf'
      )

    cr10_description = IncludeLaunchDescription(
      PythonLaunchDescriptionSource([os.path.join(
         get_package_share_directory('cr10_description'), 'launch'),
         '/display2.launch.py'])
      )
    
    forward_kinematics = Node(
         package='forward_kinematics',
         executable='forward_kinematics',
         name='ForwardKinematics',
         parameters=[{
            'urdf_path': urdf_path,
         }])
    
    trajectory_planner = Node(
         package='trajectory_planner',
         executable='trajectory_planning_linear',
         name='Planner',
         parameters=[{
            'urdf_path': urdf_path,
         }])


    return LaunchDescription([
      cr10_description,
      forward_kinematics,
      trajectory_planner,
   ])