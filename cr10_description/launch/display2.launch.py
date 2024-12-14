from ament_index_python.packages import get_package_share_path

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess
from launch.substitutions import Command, LaunchConfiguration

from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue

def generate_launch_description():
    urdf_path = get_package_share_path('cr10_description')
    default_model_path = urdf_path / f'urdf/CR10URDF.urdf'
    default_rviz_config_path = urdf_path / 'rviz/urdf.rviz'

    model_arg = DeclareLaunchArgument(name='model', default_value=str(default_model_path),
                                      description='Absolute path to robot urdf file')
    rviz_arg = DeclareLaunchArgument(name='rvizconfig', default_value=str(default_rviz_config_path),
                                     description='Absolute path to rviz config file')

    robot_description = ParameterValue(Command(['xacro ', LaunchConfiguration('model')]),
                                       value_type=str)

    robot_state_publisher_node = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        parameters=[{'robot_description': robot_description}]
    )

    # One-time joint states publisher
    publish_zeros_once = ExecuteProcess(
        cmd=[
            'ros2', 'topic', 'pub', '--once',
            '/joint_states', 'sensor_msgs/JointState',
            '{header: {stamp: now, frame_id: ""}, name: ["joint1", "joint2", "joint3", "joint4", "joint5", "joint6"], position: [0.0, 0.0, 0.0, 0.0, 0.0, 0.0]}'
        ],
        output='screen'
    )

    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        output='screen',
        arguments=['-d', LaunchConfiguration('rvizconfig')],
    )

    return LaunchDescription([
        model_arg,
        rviz_arg,
        robot_state_publisher_node,
        publish_zeros_once,
        rviz_node
    ])
