import os
from launch import LaunchDescription
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
from launch.actions import RegisterEventHandler
from launch.event_handlers import OnProcessExit
from launch.actions import ExecuteProcess

def generate_launch_description():


    jetank_package = get_package_share_directory('jetank_description')
    jetank_controllers_config = os.path.join(jetank_package,"config","jetank_controllers.yaml")

    # define the controller nodes that communicate
    # between the DDS topics and the ros2_control control manager
    # NOTE: controllers can and must not claim the same command_interfaces
    # you can find this in your YAML file that sets the configuration for these
    # controllers and which joints they have claim on
    joint_state_broadcaster_spawner = Node(
        package='controller_manager',
        executable='spawner',
        arguments=[
            'joint_state_broadcaster',
            '--switch-timeout', '10'
            ]
    )
    diff_drive_controller_spawner = Node(
        package='controller_manager',
        executable='spawner',
        arguments=[
            'diff_drive_controller',
            '--param-file',jetank_controllers_config,
            '--switch-timeout', '10'
            ]
    )
    arm_controller_spawner = Node(
        package='controller_manager',
        executable='spawner',
        arguments=[
            'arm_controller',
            '--param-file',jetank_controllers_config,
            '--switch-timeout', '10'
            ]
     )
    gripper_controller_spawner = Node(
        package='controller_manager',
        executable='spawner',
        arguments=[
            'gripper_controller',
            '--param-file',jetank_controllers_config,
            '--switch-timeout', '10'
            ]
    )

    # Start the controller manager
    # required to handle the controllers nodes I/O
    # and ros2_control resource manager
    # NOTE: due to the package 'controller_manager_node' it is not required to run this 
    # code but it could be that outside a simulation this might be required
     
    # controller_manager_node = Node(
    #     package="controller_manager",
    #     executable="ros2_control_node",
    #     parameters=[jetank_controllers_config],
    #     output="screen"
    # )

    # event handlers that will ensure that some nodes
    # are started in a lineair fashion:
    # in this case we assign a event handler on the joint_state_broadcaster
    # once the process stops it launches the controller nodes
    joint_state_event_handler = RegisterEventHandler(
        event_handler=OnProcessExit(
            target_action=joint_state_broadcaster_spawner,
            on_exit=[
                diff_drive_controller_spawner,
                arm_controller_spawner,
                gripper_controller_spawner
            ]
    ))     
   
    return LaunchDescription([
        # NOTE: this can be ignored but outside the simulation 
        # it might be neccessary
        # controller_manager_node,
        joint_state_broadcaster_spawner,
        joint_state_event_handler
    ])
