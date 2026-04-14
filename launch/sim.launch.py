#!/usr/bin/env python3
import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, SetEnvironmentVariable, DeclareLaunchArgument, ExecuteProcess
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PythonExpression
from launch_ros.actions import Node

#=====================================================================================================
#======================================= generate launch description ======================================
def generate_launch_description():

    px4_dir = os.path.expanduser('~/drone/PX4-Autopilot')
    
    # Get package directories
    pkg_surgical_drone = get_package_share_directory('surgical_drone')
    pkg_ros_gz_sim = get_package_share_directory('ros_gz_sim')
    
    # Locate the world and model files
    world_file = os.path.join(pkg_surgical_drone, 'config', 'worlds', 'surgical_drone_world.sdf')
    model_path = os.path.join(pkg_surgical_drone, 'config', 'models')
    
    config_file_path = os.path.join(pkg_surgical_drone, 'launch', 'server.config')

    # --- Launch Arguments ---
    verbose_arg = DeclareLaunchArgument('verbose', default_value='True')
    headless_arg = DeclareLaunchArgument('headless', default_value='False')

    verbose = LaunchConfiguration('verbose')
    headless = LaunchConfiguration('headless')

    # --- Environment Variables ---
    # Set Gazebo resource path so it finds your custom models
    gz_resource_path = SetEnvironmentVariable(
        name='GZ_SIM_RESOURCE_PATH', 
    	value=model_path
    )

    gz_server_config = SetEnvironmentVariable(
        name='GZ_SIM_SERVER_CONFIG_PATH',
        value=config_file_path
	)
    
#=========================================Gazebo Sim ==============================================================
    gz_sim = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            os.path.join(pkg_ros_gz_sim, 'launch', 'gz_sim.launch.py')
        ]),
        launch_arguments={
            'gz_args': PythonExpression([
                f"'{world_file} -r'",
                " + (' -v' if '", verbose, "' == 'True' else '')",
                " + (' -s' if '", headless, "' == 'True' else '')"
            ])
        }.items()
    )    
    
#=====================================================================================================
#======================================= start px4 ==============================================================
    start_px4 = ExecuteProcess(
        cmd=[
            'gnome-terminal', '--', 
            'bash', '-c', 
            f'cd {px4_dir} && make px4_sitl gz_x500'
        ],
        additional_env={
            'PX4_GZ_STANDALONE': '1',
            'PX4_GZ_WORLD': 'surgical_drone_world'
        },
        output='screen'
    )
#=====================================================================================================
#======================================= ros_gz_bridge ========================================================
    ros_gz_bridge = Node(
            package='ros_gz_bridge',
            executable='parameter_bridge',
            name='ros_gz_bridge_camera',
            arguments=[
                # ROS -> GZ (Control)
                '/cmd_vel@geometry_msgs/msg/Twist]gz.msgs.Twist',
                '/cmd_path@geometry_msgs/msg/PoseArray]gz.msgs.Pose_V',
                '/model/left_arm/joint/panda_joint1/0/cmd_pos@std_msgs/msg/Float64[gz.msgs.Double@left_arm/joint_1/cmd_pos',
                '/model/left_arm/joint/panda_joint2/0/cmd_pos@std_msgs/msg/Float64[gz.msgs.Double@left_arm/joint_2/cmd_pos',
                '/model/left_arm/joint/panda_joint3/0/cmd_pos@std_msgs/msg/Float64[gz.msgs.Double@left_arm/joint_3/cmd_pos',
                '/model/left_arm/joint/panda_joint4/0/cmd_pos@std_msgs/msg/Float64[gz.msgs.Double@left_arm/joint_4/cmd_pos',
                '/model/left_arm/joint/panda_joint5/0/cmd_pos@std_msgs/msg/Float64[gz.msgs.Double@left_arm/joint_5/cmd_pos',
                '/model/left_arm/joint/panda_joint6/0/cmd_pos@std_msgs/msg/Float64[gz.msgs.Double@left_arm/joint_6/cmd_pos',
                '/model/left_arm/joint/panda_joint7/0/cmd_pos@std_msgs/msg/Float64[gz.msgs.Double@left_arm/joint_7/cmd_pos',
                '/model/right_arm/joint/panda_joint1/0/cmd_pos@std_msgs/msg/Float64[gz.msgs.Double@right_arm/joint_1/cmd_pos',
                '/model/right_arm/joint/panda_joint2/0/cmd_pos@std_msgs/msg/Float64[gz.msgs.Double@right_arm/joint_2/cmd_pos',
                '/model/right_arm/joint/panda_joint3/0/cmd_pos@std_msgs/msg/Float64[gz.msgs.Double@right_arm/joint_3/cmd_pos',
                '/model/right_arm/joint/panda_joint4/0/cmd_pos@std_msgs/msg/Float64[gz.msgs.Double@right_arm/joint_4/cmd_pos',
                '/model/right_arm/joint/panda_joint5/0/cmd_pos@std_msgs/msg/Float64[gz.msgs.Double@right_arm/joint_5/cmd_pos',
                '/model/right_arm/joint/panda_joint6/0/cmd_pos@std_msgs/msg/Float64[gz.msgs.Double@right_arm/joint_6/cmd_pos',
                '/model/right_arm/joint/panda_joint7/0/cmd_pos@std_msgs/msg/Float64[gz.msgs.Double@right_arm/joint_7/cmd_pos',
            ]
    )

#====================================================================================================
#========================================= dds agent ===================================================
    dds_agent_start = ExecuteProcess(
        cmd=[
            'gnome-terminal', '--', 
            'bash', '-c', 
            f'MicroXRCEAgent udp4 -p 8888; exec bash'
        ],
        output='screen'
    )
#=====================================================================================================

#=====================================================================================================
#======================================= return launch description ======================================
    return LaunchDescription([
        verbose_arg,
        headless_arg,
        gz_resource_path,
        gz_server_config,
        gz_sim,
        start_px4,
        dds_agent_start,
        ros_gz_bridge,
    ])
