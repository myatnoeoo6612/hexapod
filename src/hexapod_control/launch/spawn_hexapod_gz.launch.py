# import os
# import yaml
# from launch import LaunchDescription
# from launch.actions import DeclareLaunchArgument, ExecuteProcess
# from launch.substitutions import LaunchConfiguration
# from launch_ros.actions import Node
# from launch_ros.substitutions import FindPackageShare


# def generate_launch_description():
#     # Find package paths
#     description_pkg = FindPackageShare("hexapod_description").find("hexapod_description")
#     control_pkg = FindPackageShare("hexapod_control").find("hexapod_control")

#     # File paths
#     default_model_path = os.path.join(description_pkg, "urdf", "hexapod.urdf")
#     controllers_yaml_path = os.path.join(description_pkg, "config", "controllers.yaml")
#     default_world_path = os.path.join(description_pkg, "worlds", "default.sdf")

#     # Declare arguments
#     model_arg = DeclareLaunchArgument(
#         name="model",
#         default_value=default_model_path,
#         description="Absolute path to robot URDF file"
#     )
#     world_arg = DeclareLaunchArgument(
#         name="world",
#         default_value=default_world_path,
#         description="Absolute path to world SDF file"
#     )

#     model_path = LaunchConfiguration("model")
#     world_path = LaunchConfiguration("world")

#     # Read robot description
#     with open(default_model_path, "r") as infp:
#         robot_description_content = infp.read()

#     # Load controllers.yaml params
#     with open(controllers_yaml_path, "r") as f:
#         controllers_yaml = yaml.safe_load(f)

#     # Robot State Publisher Node
#     robot_state_publisher_node = Node(
#         package="robot_state_publisher",
#         executable="robot_state_publisher",
#         output="screen",
#         parameters=[{"robot_description": robot_description_content}]
#     )

#     # Spawn entity using ros_gz_sim create
#     spawn_entity = ExecuteProcess(
#         cmd=[
#             "ros2", "run", "ros_gz_sim", "create",
#             "-name", "hexapod",
#             "-topic", "robot_description",
#             "-x", "0",
#             "-y", "0",
#             "-z", "0.2"
#         ],
#         output="screen"
#     )

#     # Controller manager node
#     controller_manager = Node(
#         package="controller_manager",
#         executable="ros2_control_node",
#         namespace="my_robot",  # <---- add this line!
#         parameters=[{"robot_description": robot_description_content}, controllers_yaml],
#         output="screen",
#         name="controller_manager"  # <-- add this line!
#     )


#     # Spawner nodes
#     joint_state_broadcaster_spawner = Node(
#         package="controller_manager",
#         executable="spawner",
#         arguments=["joint_state_broadcaster", "--controller-manager", "/controller_manager"],
#         output="screen"
#     )

#     joint_controllers = [
#         "hip_1_joint_position_controller",
#         "knee_1_joint_position_controller",
#         "ankle_1_joint_position_controller",
#         "hip_2_joint_position_controller",
#         "knee_2_joint_position_controller",
#         "ankle_2_joint_position_controller",
#         "hip_3_joint_position_controller",
#         "knee_3_joint_position_controller",
#         "ankle_3_joint_position_controller",
#         "hip_4_joint_position_controller",
#         "knee_4_joint_position_controller",
#         "ankle_4_joint_position_controller",
#         "hip_5_joint_position_controller",
#         "knee_5_joint_position_controller",
#         "ankle_5_joint_position_controller",
#         "hip_6_joint_position_controller",
#         "knee_6_joint_position_controller",
#         "ankle_6_joint_position_controller",
#     ]

#     # Spawn all joint position controllers
#     joint_position_spawners = [
#         Node(
#             package="controller_manager",
#             executable="spawner",
#             arguments=[controller_name, "--controller-manager", "/controller_manager"],
#             output="screen"
#         ) for controller_name in joint_controllers
#     ]

#     return LaunchDescription([
#         model_arg,
#         world_arg,
#         robot_state_publisher_node,
#         spawn_entity,
#         # controller_manager,
#         joint_state_broadcaster_spawner,
#         # *joint_position_spawners,
#     ])
import os
import yaml
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    # Find package paths
    description_pkg = FindPackageShare("hexapod_description").find("hexapod_description")

    # File paths
    default_model_path = os.path.join(description_pkg, "urdf", "hexapod.urdf")
    default_world_path = os.path.join(description_pkg, "worlds", "default.sdf")

    # Declare arguments
    model_arg = DeclareLaunchArgument(
        name="model",
        default_value=default_model_path,
        description="Absolute path to robot URDF file"
    )
    world_arg = DeclareLaunchArgument(
        name="world",
        default_value=default_world_path,
        description="Absolute path to world SDF file"
    )

    model_path = LaunchConfiguration("model")
    world_path = LaunchConfiguration("world")

    # Read robot description
    with open(default_model_path, "r") as infp:
        robot_description_content = infp.read()

    # Robot State Publisher Node
    robot_state_publisher_node = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        output="screen",
        parameters=[{"robot_description": robot_description_content}]
    )

    # Spawn entity using ros_gz_sim create
    spawn_entity = ExecuteProcess(
        cmd=[
            "ros2", "run", "ros_gz_sim", "create",
            "-name", "hexapod",
            "-topic", "robot_description",
            "-x", "0",
            "-y", "0",
            "-z", "0.2"
        ],
        output="screen"
    )

    return LaunchDescription([
        model_arg,
        world_arg,
        robot_state_publisher_node,
        spawn_entity,
    ])
