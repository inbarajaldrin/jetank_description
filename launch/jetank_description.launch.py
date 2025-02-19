import os
import xacro

from launch import LaunchDescription
from launch_ros.actions import Node

from ament_index_python.packages import get_package_share_directory

from launch.actions import DeclareLaunchArgument , IncludeLaunchDescription
from launch.substitutions import LaunchConfiguration
from launch.launch_description_sources import PythonLaunchDescriptionSource

def generate_launch_description():

    # get this package's location in share directory
    jetank_package = get_package_share_directory('jetank_description')
    
    # here we just declare and define a variable that will be evaluated at runtime
    # to see whether we use the simulation's clock or unix clock
    use_sim_time = LaunchConfiguration('use_sim_time')
    use_sim_time_arg = DeclareLaunchArgument(
        name='use_sim_time',
        default_value='false',
        description='[ARG] tells the robot_state_publisher to use the simulation time or just unix timestamp'
    )

    # gather the main xacro file's location in order to feed it to the 
    # Xacro program
    main_xacro_file = os.path.join(
        jetank_package,'urdf','jetank_main.xacro'
    )

    if os.path.exists(main_xacro_file):
        jetank_description = xacro.process(main_xacro_file)
    else:
        print(f'failed to open {main_xacro_file}')
        exit()

    
    return LaunchDescription([
        use_sim_time_arg,
        Node(
            package='robot_state_publisher',
            executable='robot_state_publisher',
            output='screen',
            parameters=[{
                'robot_description': jetank_description,
                'use_sim_time': use_sim_time
            }],
        ),   
    ])
        

