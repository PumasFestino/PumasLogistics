#!/usr/bin/env python3
import os
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, ExecuteProcess
from launch.substitutions import LaunchConfiguration
from launch.launch_description_sources import PythonLaunchDescriptionSource
from ament_index_python.packages import get_package_share_directory
from launch_ros.actions import Node
from launch.conditions import IfCondition  # <-- NUEVO

def generate_launch_description():
    # === For Festo Base (robotino) ===
    namespace      = DeclareLaunchArgument('namespace',       default_value='hardware/robotino',            description='Top-level namespace')
    use_sim_time   = DeclareLaunchArgument('use_sim_time',    default_value='false',       description='Use /clock if true')
    launch_jsb     = DeclareLaunchArgument('launch_jsb',      default_value='false',       description='Launch joint_state_publisher')
    hostname       = DeclareLaunchArgument('hostname',        default_value='172.27.1.1',  description='IP/hostname de Robotino')
    rsp_freq       = DeclareLaunchArgument('rsp_freq',        default_value='20.0',        description='Frec from robot_state_publisher')
    bumper_timeout = DeclareLaunchArgument('bumper_timeout',  default_value='2.0',         description='Time for bumper stop')
    motor_timeout  = DeclareLaunchArgument('motor_timeout',   default_value='0.0',         description='Timeout of motor')
    launch_odom_tf = DeclareLaunchArgument('launch_odom_tf',  default_value='true',        description='Publish TF Odom')

    # robot_description: from robotino
    default_xacro = os.path.join(
        get_package_share_directory('robotino_description'),
        'urdf', 'robotino.urdf.xacro'
    )
    robot_description = DeclareLaunchArgument(
        'robot_description', default_value=default_xacro,
        description='path to .xacro from robot'
    )

    included_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(
                get_package_share_directory('robotino_node'),
                'launch', 'robotino_driver.launch.py'
            )
        ),
        launch_arguments={
            'namespace':        LaunchConfiguration('namespace'),
            'use_sim_time':     LaunchConfiguration('use_sim_time'),
            'launch_jsb':       LaunchConfiguration('launch_jsb'),
            'robot_description':LaunchConfiguration('robot_description'),
            'hostname':         LaunchConfiguration('hostname'),
            'rsp_freq':         LaunchConfiguration('rsp_freq'),
            'bumper_timeout':   LaunchConfiguration('bumper_timeout'),
            'motor_timeout':    LaunchConfiguration('motor_timeout'),
            'launch_odom_tf':   LaunchConfiguration('launch_odom_tf'),
        }.items()
    )

    # === Navigation Launch ===
    use_nav        = DeclareLaunchArgument('use_nav',         default_value='true',        description='Launch navigation stack')
    navigation_launch = ExecuteProcess(
        cmd=[
            'ros2', 'launch', 'navigation_start', 'navigation.launch.xml',
            'namespace:=hardware/robotino'  # puedes dejarlo fijo o reemplazar por tu LaunchConfiguration si gustas
        ],
        output='screen',
        condition=IfCondition(LaunchConfiguration('use_nav'))  # <-- NUEVO
    )

    # === RViz2 Launch ===
    rviz_config = os.path.join(
        get_package_share_directory('surge_et_ambula'),
        'rviz',
        'justina.rviz'
    )

    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        output='screen',
        arguments=['-d', rviz_config],
        parameters=[{'use_sim_time': LaunchConfiguration('use_sim_time')}]
    )

    return LaunchDescription([
        namespace, use_sim_time, launch_jsb, robot_description,
        hostname, rsp_freq, bumper_timeout, motor_timeout, launch_odom_tf,
        use_nav,                         # <-- NUEVO
        included_launch,
        navigation_launch,
        rviz_node
    ])
