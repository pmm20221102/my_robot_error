import launch
import launch_ros
from ament_index_python.packages import get_package_share_directory
import os

import launch_ros.parameter_descriptions

def generate_launch_description():
    urdf_package_path = get_package_share_directory('robot_description')
    default_xacro_path = os.path.join(urdf_package_path,'urdf','robot','robot.urdf.xacro')
    default_gazebo_world_path= os.path.join(urdf_package_path,'world','room.sdf')
    gazebo_launch_file = os.path.join(get_package_share_directory('ros_gz_sim'),'launch','ros_gz_sim.launch.py')
    bridge_config_path=os.path.join(urdf_package_path,'config','bridge_config.yaml')
    action_declare_arg_mode_path = launch.actions.DeclareLaunchArgument(
        name='model',default_value = str(default_xacro_path),description='path'
    )
    substitutions_command_result = launch.substitutions.Command(['xacro ',launch.substitutions.LaunchConfiguration('model')])
    robot_description_value = launch_ros.parameter_descriptions.ParameterValue(substitutions_command_result,value_type=str)

    action_robot_state_publisher = launch_ros.actions.Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        parameters=[{'robot_description':robot_description_value}])
    
    action_launch_gazebo = launch.actions.IncludeLaunchDescription(
        launch.launch_description_sources.PythonLaunchDescriptionSource(
            gazebo_launch_file
        ),
        launch_arguments=[('world_sdf_file',default_gazebo_world_path),
                          ('bridge_name','bridge_config'),
                          ('config_file',bridge_config_path)
                          ]

    )
    #action_spawn_entity = launch_ros.actions.Node(
    #   package='ros_gz_sim',
    #   executable='create',
    #   arguments=['-name','roboter','-topic','/roboz_description']
    #)
    #action_joint_state_publisher = launch_ros.actions.Node(
    #    package='joint_state_publisher',
    #    executable='joint_state_publisher',
    #)
    #action_rviz_node = launch_ros.actions.Node(
    #   package='rviz2',
    #   executable='rviz2'
    #)
    
    return launch.LaunchDescription([
        action_declare_arg_mode_path,
        action_robot_state_publisher,
        action_launch_gazebo,
        #action_spawn_entity
        #action_joint_state_publisher,
        #action_rviz_node
    ])
