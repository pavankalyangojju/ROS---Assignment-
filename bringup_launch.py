# launch/bringup_launch.py
import os
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, ExecuteProcess
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory

def generate_launch_description():
    pkg_share = get_package_share_directory('your_package')  # change to your package name

    # paths - adjust to your package layout
    rviz_cfg = os.path.join(pkg_share, 'rviz', 'nav2_waypoints.rviz')
    map_yaml = os.path.join(pkg_share, 'config', 'map.yaml')
    nav2_params = os.path.join(pkg_share, 'config', 'nav2_params.yaml')

    # Launch Gazebo with turtlebot3 world (you can use turtlebot3_gazebo package)
    gazebo = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            os.path.join(get_package_share_directory('turtlebot3_gazebo'), 'launch', 'turtlebot3_world.launch.py')
        ])
    )

    # Launch nav2 bringup (assumes nav2_bringup is installed)
    nav2_bringup = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            os.path.join(get_package_share_directory('nav2_bringup'), 'launch', 'bringup_launch.py')
        ]),
        launch_arguments={
            'map': map_yaml,
            'params_file': nav2_params,
            'use_sim_time': 'true'
        }.items()
    )

    # Our waypoint manager node
    waypoint_manager = Node(
        package='your_package',
        executable='waypoint_manager_node',  # if installed as executable via setup.py, else use python: module path
        name='waypoint_manager',
        output='screen',
        parameters=[os.path.join(pkg_share, 'config', 'waypoints.yaml')]
    )

    # GUI node (runs on same machine; must have DISPLAY if launching GUI)
    gui_node = Node(
        package='your_package',
        executable='gui_tkinter',
        name='waypoint_gui',
        output='screen',
        emulate_tty=True,
        # GUI nodes typically require DISPLAY; if remote, use X forwarding
    )

    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        arguments=['-d', rviz_cfg],
        output='screen'
    )

    ld = LaunchDescription()
    # order: gazebo then nav2 then custom nodes
    ld.add_action(gazebo)
    ld.add_action(nav2_bringup)
    ld.add_action(rviz_node)
    ld.add_action(waypoint_manager)
    ld.add_action(gui_node)

    return ld
