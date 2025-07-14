import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, ExecuteProcess
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node
from launch.actions import TimerAction

def generate_launch_description():
    package_name = 'infinity_bot'

    # Lança o robot_state_publisher com uso de sim time
    rsp = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            os.path.join(get_package_share_directory(package_name), 'launch', 'rsp.launch.py')
        ]),
        launch_arguments={'use_sim_time': 'true'}.items()
    )

    # Lança o Gazebo Harmonic (Ignition Gazebo) com mundo customizado
    gz_sim = ExecuteProcess(
        cmd=[
            'gz', 'sim',
            os.path.join(
                get_package_share_directory(package_name),
                'worlds',
                'px4_world.sdf'
            ),
            '-r', '--verbose'
        ],
        output='screen'
    )

    # Spawner do modelo no Gazebo Harmonic via ros_gz_sim
    spawn_entity = TimerAction(
        period=5.0,
        actions=[Node(
            package='ros_gz_sim',
            executable='create',
            arguments=[
                '-name', 'my_bot',
                '-topic', 'robot_description',
                '-z', '0.4'
            ],
            output='screen'
        )]
    )


    scan_bridge = Node(
        package='ros_gz_bridge',
        executable='parameter_bridge',
        arguments=[
            '/scan@sensor_msgs/msg/LaserScan@gz.msgs.LaserScan'
        ],
        output='screen'
    )
        
    odom_bridge = Node(
        package='ros_gz_bridge',
        executable='parameter_bridge',
        arguments=[
            '/odom@nav_msgs/msg/Odometry@gz.msgs.Odometry'
        ],
        output='screen'
    )
        
    points_bridge = Node(
        package='ros_gz_bridge',
        executable='parameter_bridge',
        arguments=[
            '/scan/points@sensor_msgs/msg/PointCloud2@gz.msgs.PointCloudPacked'
        ],
        output='screen'
    )


    # Bridge para o tópico /cmd_vel entre ROS 2 e Gazebo
    cmd_vel_bridge = Node(
        package='ros_gz_bridge',
        executable='parameter_bridge',
        arguments=[
            '/cmd_vel@geometry_msgs/msg/Twist@gz.msgs.Twist'
        ],
        output='screen'
    )

    imu_bridge = Node(
        package='ros_gz_bridge',
        executable='parameter_bridge',
        arguments=[
            'imu@sensor_msgs/msg/Imu@gz.msgs.IMU'
        ],
        output='screen'
    )

    # teleop = Node(
    #     package='teleop_twist_keyboard',
    #     executable='teleop_twist_keyboard',
    #     name='teleop_twist_keyboard',
    #     output='screen',
    #     remappings=[('/cmd_vel', '/cmd_vel')]
    # )

    return LaunchDescription([
        rsp,
        gz_sim,
        spawn_entity,
        cmd_vel_bridge,
        scan_bridge,
        odom_bridge,
        points_bridge,
        imu_bridge,
        # teleop
    ])
