from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    return LaunchDescription([
        Node(
            package='robot_task_manager',
            executable='robot_task_manager',
            name='robot_task_manager',
            output='screen',
            parameters=[{
                'trajectory_file': '/home/aaa/ros2_project_data/matlab/trajectory/trajectory_log_6dof.csv'
            }]
        ),
        Node(
            package='robot_task_manager',
            executable='joint_ref_bridge',
            name='joint_ref_bridge',
            output='screen'
        ),
        Node(
            package='robot_task_manager',
            executable='pose_web_gui',
            name='pose_web_gui',
            output='screen'
        ),
    ])
