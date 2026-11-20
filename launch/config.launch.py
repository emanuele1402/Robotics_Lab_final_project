import os
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, DeclareLaunchArgument, RegisterEventHandler, SetEnvironmentVariable
from launch.event_handlers import OnProcessExit
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import Command, LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
from ament_index_python.packages import get_package_share_directory
from launch_ros.parameter_descriptions import ParameterValue

def generate_launch_description():

    # Argomenti dichiarati
    declared_arguments = [
        DeclareLaunchArgument('command_interface', default_value='position'),
        DeclareLaunchArgument('robot_controller', default_value='kuka_arm_controller')
    ]
    
    robot_controller = LaunchConfiguration('robot_controller')
    
    # Percorsi
    pkg_final_project = FindPackageShare('final_project')
    world_file = PathJoinSubstitution([pkg_final_project, 'worlds', 'nastro.world'])
    kuka_urdf = PathJoinSubstitution([pkg_final_project, 'urdf', 'kuka_robot.urdf.xacro'])
    piston_urdf = PathJoinSubstitution([pkg_final_project, 'urdf', 'piston.urdf'])
    amr_urdf_path = PathJoinSubstitution([pkg_final_project, 'urdf', 'amr.urdf.xacro'])
    models_path = os.path.join(get_package_share_directory('final_project'), 'models')
    

    # --- Robot Description ---
    robot_description_kuka = {'robot_description': Command(['xacro ', kuka_urdf])} 
    
    rsp_kuka = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        name='rsp_kuka',
        namespace='kuka_ns',
        output='both',
        parameters=[robot_description_kuka, {'use_sim_time': True,"frame_prefix": "kuka/"}],
    )

    robot_description_piston = {'robot_description': Command(['cat ', piston_urdf])}
    rsp_piston = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        name='rsp_piston',
        namespace='piston_ns',
        output='both',
        parameters=[robot_description_piston, {'use_sim_time': True,"frame_prefix": "piston/"}],
    )
    robot_description_amr = {'robot_description': Command(['xacro ', amr_urdf_path])} 
    rsp_amr = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        name='rsp_amr',
        namespace='amr',
        output='screen',
        parameters=[robot_description_amr,{
            'use_sim_time': True,
            'frame_prefix': 'amr/'
        }]
    )
    jsp_amr= Node(
         package='joint_state_publisher',
         executable='joint_state_publisher',
         namespace='amr',
         parameters=[{"use_sim_time": True}]
     )

    # --- Gazebo ---
    gazebo = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            [PathJoinSubstitution([FindPackageShare('ros_ign_gazebo'), 'launch', 'ign_gazebo.launch.py'])]
        ),
        launch_arguments={'gz_args': ['-r -v 4 ', world_file]}.items()
    ) 
    
    # --- Spawn Robots ---
    spawn_kuka = Node(
        package='ros_gz_sim',
        executable='create',
        arguments=[
            '-name', 'kuka_robot',
            '-topic', '/kuka_ns/robot_description',
            '-x', '1.6', '-y', '1.2', '-z', '0.05',
            '-R', '0.0', '-P', '0.0', '-Y', '1.5708'
        ],
        output='screen'
    )

    spawn_piston = Node(
        package='ros_gz_sim',
        executable='create',
        arguments=[
            '-name', 'piston_pneumatic',
            '-topic', '/piston_ns/robot_description',
            '-x', '2.3', '-y', '-0.65', '-z', '0.1',
            '-R', '0.0', '-P', '0.0', '-Y', '1.5708'
        ],
        output='screen'
    )
    spawn_amr = Node(
        package='ros_gz_sim',
        executable='create',
        name='spawn_amr',
        arguments=[ 
            '-name', 'amr_robot', 
            '-topic', '/amr/robot_description',
            '-x', '2.4', '-y', '0.85', '-z', '0.1', # x=2.4 y=0.8
            
        ],
        output='screen'
    )

    # --- Bridge ---
    bridge = Node(
        package='ros_gz_bridge',
        executable='parameter_bridge',
        arguments=[
            '/model/box1/cmd_vel@geometry_msgs/msg/Twist@ignition.msgs.Twist',
            '/model/box2/cmd_vel@geometry_msgs/msg/Twist@ignition.msgs.Twist',
            '/lidar_scan@sensor_msgs/msg/LaserScan[ignition.msgs.LaserScan',
            '/amr/lidar@sensor_msgs/msg/LaserScan[ignition.msgs.LaserScan',
            '/clock@rosgraph_msgs/msg/Clock[ignition.msgs.Clock',
            '/kuka_ns/joint_states@sensor_msgs/msg/JointState[ignition.msgs.JointState',
            '/piston_ns/joint_states@sensor_msgs/msg/JointState[ignition.msgs.JointState',
            '/kuka/camera1/image_raw@sensor_msgs/msg/Image[ignition.msgs.Image',
            '/kuka/camera1/camera_info@sensor_msgs/msg/CameraInfo[ignition.msgs.CameraInfo',
            '/world/conveyor_world/set_pose@ros_gz_interfaces/srv/SetEntityPose',
            '/cmd_vel@geometry_msgs/msg/Twist@ignition.msgs.Twist',
            '/model/amr_robot/odometry@nav_msgs/msg/Odometry[ignition.msgs.Odometry',
            '/model/amr_robot/tf@tf2_msgs/msg/TFMessage[ignition.msgs.PoseBundle',
            
            
        ],
        remappings=[
            ('/model/amr_robot/tf', '/tf')
        ],
        output='screen'
    )
    
    

    # --- Controller Spawners ---
    kuka_jsb_spawner = Node(
        package='controller_manager',
        executable='spawner',
        arguments=['joint_state_broadcaster', '--controller-manager', '/kuka_ns/controller_manager'],
    )

    robot_controller_spawner_kuka = Node(
       package='controller_manager',
       executable='spawner',
       arguments=[robot_controller, '--controller-manager', '/kuka_ns/controller_manager'], 
    )

    piston_jsb_spawner = Node(
        package='controller_manager',
        executable='spawner',
        arguments=['joint_state_broadcaster', '--controller-manager', '/piston_ns/controller_manager'],
    )

    piston_controller_spawner = Node(
        package='controller_manager',
        executable='spawner',
        arguments=['piston_controller', '--controller-manager', '/piston_ns/controller_manager'],
        output='screen',
    )
    
    
    # --- Event Handlers (Execution Order) ---
    delay_kuka_jsb = RegisterEventHandler(
        event_handler=OnProcessExit(target_action=spawn_kuka, on_exit=[kuka_jsb_spawner])
    )

    delay_robot_controller_kuka = RegisterEventHandler(
        event_handler=OnProcessExit(target_action=kuka_jsb_spawner, on_exit=[robot_controller_spawner_kuka])
    )

    delay_piston_jsb = RegisterEventHandler(
        event_handler=OnProcessExit(target_action=spawn_piston, on_exit=[piston_jsb_spawner])
    )
    
    delay_piston_controller = RegisterEventHandler(
        event_handler=OnProcessExit(target_action=piston_jsb_spawner, on_exit=[piston_controller_spawner])
    )
   
   
    odom_tf = Node(
        package='final_project',
        executable='dynamic_tf_publisher',
        name='odom_tf',
        parameters=[{"use_sim_time": True}]
    )
    
    
    
    nodes_to_start = [gazebo,
        bridge,
        rsp_kuka,
        rsp_piston,
        rsp_amr,
        jsp_amr,
        spawn_kuka,
        spawn_piston,
        spawn_amr,
        delay_kuka_jsb,           
        delay_robot_controller_kuka,
        delay_piston_jsb,         
        delay_piston_controller,
        odom_tf,
        
         
        ]
    return LaunchDescription([SetEnvironmentVariable(name="GZ_SIM_RESOURCE_PATH", value = models_path + ':' + os.environ.get('GZ_SIM_RESOURCE_PATH', ''))]+declared_arguments + nodes_to_start)
