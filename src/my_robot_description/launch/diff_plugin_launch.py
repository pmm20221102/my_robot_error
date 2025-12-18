import launch
import launch_ros
from ament_index_python.packages import get_package_share_directory
import os
import launch_ros.parameter_descriptions
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration, TextSubstitution
from ros_gz_bridge.actions import RosGzBridge

def generate_launch_description():
    #获取功能包的share路径
    urdf_package_path = get_package_share_directory('my_robot_description')
    default_xacro_path = os.path.join(urdf_package_path,'urdf', 'robot/robot.urdf.xacro')
    #default_rviz_config_path =os.path.join(urdf_package_path,"config", "display_robot_model.rviz")
    default_gz_world_path = os.path.join(urdf_package_path,'world','robot_haus.sdf')
    #default_gz_world_path = '/home/w/fish_ros2/chapt6/chapt6_ws/install/fishbot_description/share/fishbot_description/world/custom.sdf)'
    default_bridge_yaml_path = os.path.join(urdf_package_path,'config','gz_bridge.yaml')
    #声明一个urdf目录的参数，方便修改
    action_declare_arg_mode_path = launch.actions.DeclareLaunchArgument(
        name='model',default_value=str(default_xacro_path),description='path'
    )
    #通过文件路径，获取内容，并转换成参数值对象，以供传入 robot_state_publisher
    substitutions_command_result = launch.substitutions.Command(['xacro ',launch.
    substitutions.LaunchConfiguration('model')])
    robot_description_value = launch_ros.parameter_descriptions.ParameterValue(substitutions_command_result,value_type=str)
    #robot_description_value = open(default_xacro_path).read()
    action_robot_state_publisher = launch_ros.actions.Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        parameters=[{'robot_description':robot_description_value}]
    )

    action_launch_gz = launch.actions.IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            [get_package_share_directory('ros_gz_sim'),'/launch','/gz_sim.launch.py']
        ),
       
        launch_arguments={'gz_args':f'-r {default_gz_world_path}',}.items()
        #launch_arguments=[('world',default_gz_world_path),('verbose','true')]
    )

    action_spawn_entity = launch_ros.actions.Node(
        package='ros_gz_sim',
        executable='create',
        arguments=[
            '--name', 'robot',
            '--topic', '/robot_description'
        ],
        output='screen'
    )

    bridge_name = LaunchConfiguration('bridge_name')
    config_file = LaunchConfiguration('config_file')

    bridge_name = DeclareLaunchArgument(
        'bridge_name',default_value='robot_bridge',
        description='Name of ros_gz_bridge node'
    )

    config_file = DeclareLaunchArgument(
        'config_file',default_value=default_bridge_yaml_path,
        description='YAML config file'
    )
    bridge =  RosGzBridge(
            bridge_name=LaunchConfiguration('bridge_name'),
            config_file=LaunchConfiguration('config_file'),
        )
    
    joint_state_pub_node = launch_ros.actions.Node(
        package='joint_state_publisher_gui',
        executable='joint_state_publisher_gui',
    )
    static_pub_node =launch_ros.actions.Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        arguments=['0','0','0','0','0','0','laser_link','robot/base_footprint/gpu_lidar']
    )
    static_pub_imu_node =launch_ros.actions.Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        arguments=['0','0','0','0','0','0','imu_link','robot/base_footprint/imu_sensor']
    )
    static_pub_camera_node =launch_ros.actions.Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        arguments=['0','0','0','0','0','0','camera_link','robot/base_footprint/camera']
    )
    
    # Create the launch description and populate
    ld = launch.LaunchDescription()

    # Declare the launch options

    ld.add_action(action_declare_arg_mode_path)
    ld.add_action(action_robot_state_publisher)
    ld.add_action(joint_state_pub_node)
    ld.add_action(action_launch_gz)
    ld.add_action(action_spawn_entity)
    ld.add_action(static_pub_node)
    ld.add_action(static_pub_imu_node)
    ld.add_action(static_pub_camera_node)
    ld.add_action(bridge_name)
    ld.add_action(config_file)
    ld.add_action(bridge)
    

    return ld
  
