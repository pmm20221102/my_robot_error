from ament_index_python.packages import get_package_share_path
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch_ros.parameter_descriptions import ParameterValue
from launch.substitutions import Command, LaunchConfiguration
from launch_ros.actions import Node
import os
from launch.actions import ExecuteProcess
from launch.event_handlers import OnProcessExit
from launch.actions import RegisterEventHandler
import launch
from launch.actions import TimerAction

def generate_launch_description():

    robot_description_path = get_package_share_path('my_robot_description')

    # -----------------------------
    # Declare gazebo world argument
    # -----------------------------
    declare_world_arg = DeclareLaunchArgument(
        "world",
        default_value=os.path.join(robot_description_path, "world", "robot_haus.sdf"),
        description="Path to the Gazebo SDF world file"
    )
    world = LaunchConfiguration("world")

    # -----------------------------
    # Robot description (xacro)
    # -----------------------------
    urdf_path = os.path.join(robot_description_path, 'urdf', 'robot', 'robot.urdf.xacro')
#     action_declare_arg_mode_path = launch.actions.DeclareLaunchArgument(
#     name='model',
#     default_value=str(urdf_path),
#     description='加载的模型文件路径'
# )


    robot_description = ParameterValue(
        Command(["xacro ", urdf_path]),
        value_type=str
    )
#     substitutions_command_result = launch.substitutions.Command(
#     ['cat ', launch.substitutions.LaunchConfiguration('model')]
# )

#     robot_description_value = ParameterValue(
#     substitutions_command_result,
#     value_type=str
# )

    robot_controllers = os.path.join(robot_description_path, 'config', 'robot_ros2_controller.yaml')

    robot_state_publisher_node = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        parameters=[{'robot_description': robot_description}],
        output="screen",
    )

    # -----------------------------
    # ROS2 Control Node
    # -----------------------------
    # control_node = Node(
    #     package="controller_manager",
    #     executable="ros2_control_node",
    #     parameters=[
    #     robot_controllers,
    #     {"use_sim_time": True}
    # ],
    #     output="screen",
    # )
    action_load_joint_state_controller = launch.actions.ExecuteProcess(
        cmd='ros2 control load_controller joint_state_broadcaster --set-state active'.split(' '),
        output='screen'
        )


    joint_state_broadcaster_spawner = Node(
        package='controller_manager',
        executable='spawner',
        arguments=[
            'joint_state_broadcaster'],
    )

    diff_drive_controller_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=[
            "diff_drive_controller",
            '--param-file',robot_controllers,
        ],
        output="screen",
    )

    # -----------------------------
    # Start Gazebo (gz sim)
    # -----------------------------
    gazebo_node = ExecuteProcess(
    cmd=["gz", "sim", "-r", "-v4",world],
    output="screen"
)

    # -----------------------------
    # Spawn robot in Gazebo
    # -----------------------------
    spawn_node = Node(
        package="ros_gz_sim",
        executable="create",
        name="spawn_robot",
        output="screen",
        arguments=[
            "-string", Command(["xacro ", urdf_path]),
            "-name", "robot-1"
        ],
    )

    # -----------------------------
    # Gazebo ↔ ROS bridge
    # -----------------------------
    bridge_yaml = os.path.join(robot_description_path, "config", "gz_bridge.yaml")

    bridge_node = Node(
        package="ros_gz_bridge",
        executable="parameter_bridge",
        parameters=[{"config_file": bridge_yaml}],
        output="screen",
    )
    event_after_spawn = RegisterEventHandler(
        event_handler=OnProcessExit(
            target_action=spawn_node,
            on_exit=[
                joint_state_broadcaster_spawner,
                diff_drive_controller_spawner,
            ]
        )
    )
    return LaunchDescription([
        declare_world_arg,
        # action_declare_arg_mode_path,
        robot_state_publisher_node,
        gazebo_node,
        spawn_node,
        bridge_node,
        # control_node,
        event_after_spawn,
        
        # event_after_spawn,
    ])
