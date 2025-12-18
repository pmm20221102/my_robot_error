# my_robot_error
error about ros2-control jointstatebroadcaster in gz-sim
environment: ubuntu24.04+jazzy
just use ros2-Package  my_robot_description

Error Description
[gazebo-1] [INFO] [1766088901.280423299] [controller_manager]: Loading controller 'joint_state_broadcaster'
[gazebo-1] [INFO] [1766088901.287651126] [controller_manager]: Controller 'joint_state_broadcaster' node arguments: --ros-args --params-file -p use_sim_time:=true --param use_sim_time:=true
[gazebo-1] [ERROR] [1766088901.288351956] [controller_manager]: Caught exception of type : N6rclcpp10exceptions22RCLInvalidROSArgsErrorE while initializing controller 'joint_state_broadcaster': failed to parse arguments: Couldn't parse params file: '--params-file -p'. Error: Error opening YAML file, at ./src/parser.c:271, at ./src/rcl/arguments.c:415
[spawner-5] [FATAL] [1766088901.291465269] [spawner_joint_state_broadcaster]: Failed loading controller joint_state_broadcaster

relative file
config file --> my_robot_description/config/robot_ros2_controller.yaml
launch file --> launch.py (written by myself)
            --> gz_sim_launch.py (Mimicking the diff-control syntax in the official gz-ros2-control-demos)
xacro file  --> my_robot_description/urdf/robot/robot_ros2_control.xacro (This introduces gz-ros2-control)
robot.urdf.xacre --> my_robot_description/urdf/robot/robot.urdf.xacro (The main xacro file)
