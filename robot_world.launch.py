import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import ExecuteProcess, TimerAction
from launch_ros.actions import Node
from launch.substitutions import Command

def generate_launch_description():
    # Get package share directory
    pkg = get_package_share_directory('second_package')
    
  
    world_file = os.path.join(pkg, 'worlds', 'guardian_world.world')
    urdf_file = os.path.join(pkg, 'urdf', 'guardian_robot.urdf.xacro')
    
   
    if not os.path.exists(world_file):
        raise Exception(f"World file not found: {world_file}")
    if not os.path.exists(urdf_file):
        raise Exception(f"URDF file not found: {urdf_file}")

    
    gazebo = ExecuteProcess(
        cmd=['gazebo', '--verbose', world_file,
             '-s', 'libgazebo_ros_init.so',
             '-s', 'libgazebo_ros_factory.so'],
        output='screen'
    )

   
    state_pub = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        name='robot_state_publisher',
        output='screen',
        parameters=[{'robot_description': Command(['xacro ', urdf_file])}]
    )

    
    spawn = TimerAction(
        period=3.0,
        actions=[Node(
            package='gazebo_ros',
            executable='spawn_entity.py',
            arguments=[
                '-entity', 'guardian_robot',
                '-topic', 'robot_description',
                '-x', '0.0', '-y', '0.0', '-z', '0.3'
            ],
            output='screen'
        )]
    )

   
    driver = TimerAction(
        period=5.0,
        actions=[Node(
            package='second_package',
            executable='patrol_driver_node',
            output='screen'
        )]
    )

    return LaunchDescription([
        gazebo,
        state_pub,
        spawn,
        driver
    ])
