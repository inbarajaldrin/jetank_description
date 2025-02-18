from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import ExecuteProcess
import os

def generate_launch_description():


    jetank_package = get_package_share_directory('jetank_description')
    jetank_controllers_config = os.path.join(jetank_package,"config","jetank_controllers.yaml")

    # define the controller nodes that communicate
    # between the DDS topics and the ros2_control control manager
    # NOTE: controllers can and must not claim the same command_interfaces
    # you can find this in your YAML file
    joint_state_broadcaster_spawner = Node(
        package='controller_manager',
        executable='spawner',
        arguments=['joint_state_broadcaster']
    )
    diff_drive_controller_spawner = Node(
        package='controller_manager',
        executable='spawner',
        arguments=['diff_drive_controller','--param-file',jetank_controllers_config]
    )
    arm_controller_spawner = Node(
        package='controller_manager',
        executable='spawner',
        arguments=['arm_controller','--param-file',jetank_controllers_config]
    )
    gripper_controller_spawner = Node(
        package='controller_manager',
        executable='spawner',
        arguments=['gripper_controller','--param-file',jetank_controllers_config]
    )

    # Start the controller manager
    # required to handle the controllers nodes I/O
    # and ros2_control resource manager
    controller_manager_node = Node(
        package="controller_manager",
        executable="ros2_control_node",
        parameters=[jetank_controllers_config],
        output="screen"
    )
   
    return LaunchDescription([
        joint_state_broadcaster_spawner,
        diff_drive_controller_spawner,
        arm_controller_spawner,
        gripper_controller_spawner,
        controller_manager_node,
    ])

