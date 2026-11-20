import os
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, RegisterEventHandler, TimerAction
from launch.event_handlers import OnProcessExit
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory

def generate_launch_description():
    pkg_share = get_package_share_directory('final_project')
    params_file = os.path.join(pkg_share, 'config', 'kuka_params.yaml')
    
    aruco_marker_publisher_params = {
        'image_is_rectified': True,
        'marker_size': 0.2,
        'reference_frame': 'kuka/base_link',
        'camera_frame': 'kuka/camera_link_optical',
    }
    
    conveyor_controller = Node(
        package='final_project',
        executable='conveyor_controller_node',
        name='conveyor_controller',
        output='screen',
        parameters=[{'use_sim_time': True}]
    )


    server_node = Node(
        package='final_project',
        executable='kuka_action_server',
        name='kuka_action_server',
        namespace='kuka_ns',
        output='screen',
        parameters=[params_file]
    )

    aruco_node = Node(
        package='aruco_ros',
        executable='marker_publisher',
        name='marker_publisher',
        output='screen',
        parameters=[aruco_marker_publisher_params],
        remappings=[('/camera_info', '/kuka/camera1/camera_info'),
                    ('/image', '/kuka/camera1/image_raw')],
    )

    client_node = Node(
        package='final_project',
        executable='kuka_action_client',
        name='kuka_action_client',
        namespace='kuka_ns',
        output='screen'
    )
    delayed_nodes = TimerAction(
        period=2.0,  
        actions=[
            server_node,
            aruco_node,
            client_node
        ]
    )

   

    return LaunchDescription([
        conveyor_controller,
        delayed_nodes,
    ])
