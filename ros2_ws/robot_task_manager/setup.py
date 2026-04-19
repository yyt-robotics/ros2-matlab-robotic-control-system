from setuptools import find_packages, setup

package_name = 'robot_task_manager'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        (
            'share/ament_index/resource_index/packages',
            ['resource/' + package_name]
        ),
        (
            'share/' + package_name,
            ['package.xml']
        ),
	(
            'share/' + package_name + '/launch',
            ['launch/system.launch.py']
        ),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Yantong Yang',
    maintainer_email='yantongyang0303@163.com',
    description='ROS2-based robotic task execution system with trajectory generation and MATLAB/Simulink integration',
    license='MIT',
    extras_require={
        'test': ['pytest'],
    },
    entry_points={
        'console_scripts': [
            'robot_task_manager = robot_task_manager.robot_task_manager:main',
            'pose_web_gui = robot_task_manager.pose_web_gui:main',
            'joint_ref_bridge = robot_task_manager.joint_ref_bridge:main',
            'joint_state_bridge = robot_task_manager.joint_state_publisher_node:main',
        ],
    },
)
