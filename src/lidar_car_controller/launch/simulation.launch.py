from launch import LaunchDescription
from launch.actions import (IncludeLaunchDescription, TimerAction,
                            DeclareLaunchArgument)
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch.launch_description_sources import PythonLaunchDescriptionSource
from ament_index_python.packages import get_package_share_directory
import os, xacro


def generate_launch_description():
    pkg_desc   = get_package_share_directory('lidar_car_description')
    pkg_ctrl   = get_package_share_directory('lidar_car_controller')
    gazebo_ros = get_package_share_directory('gazebo_ros')

    # ── Launch arguments ───────────────────────────────────────
    use_vfh_arg = DeclareLaunchArgument(
        'use_vfh', default_value='true',
        description='Use VFH controller (true) or simple controller (false)')
    use_vfh = LaunchConfiguration('use_vfh')

    world_arg = DeclareLaunchArgument(
        'world', default_value='maze_world.world',
        description='Gazebo world file name (inside worlds/)')
    world_name = LaunchConfiguration('world')

    # ── Robot description ──────────────────────────────────────
    xacro_file = os.path.join(pkg_desc, 'urdf', 'lidar_car.urdf.xacro')
    robot_description = xacro.process_file(xacro_file).toxml()

    # ── Gazebo ─────────────────────────────────────────────────
    world_path = os.path.join(pkg_ctrl, 'worlds', 'maze_world.world')
    gazebo = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(gazebo_ros, 'launch', 'gazebo.launch.py')),
        launch_arguments={
            'world': world_path,
            'verbose': 'false',
        }.items()
    )

    # ── Robot State Publisher ──────────────────────────────────
    rsp = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        parameters=[{'robot_description': robot_description,
                     'use_sim_time': True}],
        output='screen'
    )

    # ── Spawn robot ────────────────────────────────────────────
    spawn = Node(
        package='gazebo_ros',
        executable='spawn_entity.py',
        arguments=['-topic', 'robot_description',
                   '-entity', 'lidar_car',
                   '-x', '0.0', '-y', '0.0', '-z', '0.1'],
        output='screen'
    )

    # ── VFH Controller (default) ───────────────────────────────
    vfh = Node(
    package='lidar_car_controller',
    executable='vfh_controller',
    output='screen',
    parameters=[{
        'use_sim_time':     True,
        'threshold':        10.0,
        'obstacle_dist':    2.5,
        'max_linear':       0.22,
        'max_angular':      0.65,
        'valley_min':       2,
        'min_range_cutoff': 0.11,   # ← was missing, caused param set failure
    }]
)

    # ── Path tracer ────────────────────────────────────────────
    path_tracer = Node(
        package='lidar_car_controller',
        executable='path_tracer',
        output='screen',
        parameters=[{'use_sim_time': True}]
    )

    # ── RViz2 with pre-loaded config ───────────────────────────
    rviz_config = os.path.join(pkg_desc, 'rviz', 'lidar_car.rviz')
    rviz = Node(
        package='rviz2',
        executable='rviz2',
        arguments=['-d', rviz_config],
        parameters=[{'use_sim_time': True}],
        output='screen'
    )

    return LaunchDescription([
        use_vfh_arg,
        world_arg,
        gazebo,
        rsp,
        TimerAction(period=2.0, actions=[spawn]),
        TimerAction(period=4.0, actions=[vfh, path_tracer]),
        TimerAction(period=5.0, actions=[rviz]),
    ])