import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import ExecuteProcess
from launch_ros.actions import Node

def generate_launch_description():
    # --- Find Packages ---
    pkg_hexapod_description = get_package_share_directory('hexapod_description')
    pkg_hexapod_control = get_package_share_directory('hexapod_control')  # control configs

    # --- Paths to Files ---
    urdf_file = os.path.join(pkg_hexapod_description, 'urdf', 'hexapod.urdf')
    controllers_file = os.path.join(pkg_hexapod_control, 'config', 'hexapod_controllers.yaml')

    with open(urdf_file, 'r') as f:
        robot_description_content = f.read()

    # --- Nodes ---
    robot_state_publisher_node = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        output='screen',
        parameters=[
            {'use_sim_time': True},
            {'robot_description': robot_description_content}
        ]
    )

    # controller_manager with YAML loaded
    controller_manager_node = Node(
        package='controller_manager',
        executable='ros2_control_node',
        parameters=[
            {'use_sim_time': True},
            {'robot_description': robot_description_content},
            controllers_file
        ],
        output='screen'
    )

    # Spawn robot into Gazebo
    spawn_entity = ExecuteProcess(
        cmd=[
            'ros2', 'run', 'ros_gz_sim', 'create',
            '-topic', 'robot_description',
            '-name', 'hexapod',
            '-allow_renaming', 'true',
            '-x', '0', '-y', '0', '-z', '0.2'
        ],
        output='screen'
    )


    # Spawner nodes
    controller_names_to_spawn = [
        "joint_state_broadcaster",
        "hexapod_joint_trajectory_controller",
    ]
    
    spawner_nodes = [
        Node(
            package='controller_manager',
            executable='spawner',
            arguments=[controller],
            output='screen',
        ) for controller in controller_names_to_spawn
    ]



    # Launch
    ld = LaunchDescription()
    ld.add_action(robot_state_publisher_node)
    ld.add_action(controller_manager_node)
    ld.add_action(spawn_entity)
    for spawner in spawner_nodes:
        ld.add_action(spawner)
    return ld
