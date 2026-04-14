#!/usr/bin/env python3
from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    # Bridge joint position command topics between ROS 2 and Gazebo
    joint_topics = [
        '/model/left_arm/joint/panda_joint1/0/cmd_pos',
        '/model/left_arm/joint/panda_joint2/0/cmd_pos',
        '/model/left_arm/joint/panda_joint3/0/cmd_pos',
        '/model/left_arm/joint/panda_joint4/0/cmd_pos',
        '/model/left_arm/joint/panda_joint5/0/cmd_pos',
        '/model/left_arm/joint/panda_joint6/0/cmd_pos',
        '/model/left_arm/joint/panda_joint7/0/cmd_pos',
        '/model/right_arm/joint/panda_joint1/0/cmd_pos',
        '/model/right_arm/joint/panda_joint2/0/cmd_pos',
        '/model/right_arm/joint/panda_joint3/0/cmd_pos',
        '/model/right_arm/joint/panda_joint4/0/cmd_pos',
        '/model/right_arm/joint/panda_joint5/0/cmd_pos',
        '/model/right_arm/joint/panda_joint6/0/cmd_pos',
        '/model/right_arm/joint/panda_joint7/0/cmd_pos',
    ]

    bridge_arguments = [
        f'{topic}@std_msgs/msg/Float64]gz.msgs.Double' for topic in joint_topics
    ]

    ros_gz_bridge = Node(
        package='ros_gz_bridge',
        executable='parameter_bridge',
        name='ros_gz_bridge_joint_commands',
        output='screen',
        arguments=bridge_arguments,
        parameters=[{'lazy': True}],
    )

    return LaunchDescription([ros_gz_bridge])
