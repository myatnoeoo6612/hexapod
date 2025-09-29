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

    spawn_entity = ExecuteProcess(
        cmd=[
            'ros2', 'run', 'ros_gz_sim', 'create',
            '-topic', 'robot_description',
            '-name', 'hexapod',
            '-allow_renaming', 'true',
            '-x', '0', '-y', '0', '-z', '0.5',
            '-R', '0', '-P', '0', '-Y', '0'
        ],
        output='screen'
    )

    # --- Bridge for dynamic_pose/info ---
    gz_bridge = ExecuteProcess(
        cmd=[
            'ros2', 'run', 'ros_gz_bridge', 'parameter_bridge',
            '/world/default/dynamic_pose/info@geometry_msgs/msg/PoseArray@gz.msgs.Pose_V'
        ],
        output='screen'
    )

    # --- IMU node ---
    imu_node = Node(
        package='hexapod_control',
        executable='imu',
        name='imu',
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

    # --- Launch ---
    ld = LaunchDescription()
    ld.add_action(robot_state_publisher_node)
    ld.add_action(controller_manager_node)
    ld.add_action(spawn_entity)
    ld.add_action(gz_bridge)
    ld.add_action(imu_node)
    for spawner in spawner_nodes:
        ld.add_action(spawner)
    return ld
