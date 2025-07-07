import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, ExecuteProcess
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node

def generate_launch_description():

    package_name = 'infinity_bot'  # <--- Atualize conforme o nome correto

    # Lança o rsp.launch.py (robot_state_publisher)
    rsp = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([os.path.join(
            get_package_share_directory(package_name), 'launch', 'rsp.launch.py'
        )]),
        launch_arguments={'use_sim_time': 'true'}.items()
    )

    # Lança o Gazebo Harmonic (Ignition) com ros_gz
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


    # Spawner via ros_gz service (para publicar o modelo no Gazebo Harmonic)
    spawn_entity = Node(
        package='ros_gz_sim',  # NOTA: este pacote substitui gazebo_ros para ignition
        executable='create',
        arguments=[
            '-name', 'my_bot',
            '-topic', 'robot_description',
            '-z', '0.2'
        ],
        output='screen'
    )

    return LaunchDescription([
        rsp,
        gz_sim,
        spawn_entity
    ])
