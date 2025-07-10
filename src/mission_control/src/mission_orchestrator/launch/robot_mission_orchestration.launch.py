import os
from launch import LaunchDescription
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource

def generate_launch_description():

    camera_dir = get_package_share_directory('camera_bringup')
    camera_launch_dir = os.path.join(camera_dir,'launch')

    nav2_dir = get_package_share_directory('wheeltec_nav2')
    nav2_launch_dir = os.path.join(nav2_dir,'launch')

    return LaunchDescription([
 
        # mission_orchestrator_node
        # Node(
        #     package='mission_orchestrator',
        #     executable='mission_orchestrator_node',
        #     name='mission_orchestrator',
        #     output='screen'
        # ),

        # former_yolo_node
        # Node(
        #     package='yolo_detector',
        #     executable='former_yolo_node',
        #     name='former_yolo_node',
        #     output='screen'
        # ),

        # # back_yolo_node
        # Node(
        #     package='yolo_detector',
        #     executable='back_yolo_node',
        #     name='back_yolo_node',
        #     output='screen'
        # ),

        # stm32_serial_node
        Node(
            package='stm32_map',
            executable='stm32_serial_node',
            name='stm32_serial_node',
            output='screen'
        ),

        # map_switch_node
        Node(
            package='stm32_map',
            executable='map_switch_node',
            name='map_switch_node',
            output='screen'
        ),

        # extract_location_srv_node
        Node(
            package='deepseek_chat_service',
            executable='extract_location_srv_node',
            name='extract_location_srv_node',
            output='screen'
        ),

        # azhsxf_mic (语音唤醒模块)
        Node(
            package='mic_ros2',
            executable='azhsxf_mic',
            name='azhsxf_mic',
            output='screen'
        ),

        # wheeltec_aiui (AIUI语音服务模块)
        Node(
            package='wheeltec_aiui',
            executable='wheeltec_aiui',
            name='wheeltec_aiui',
            output='screen'
        ),
        Node(
            package='go_straight',
            executable='go_straight_act_node',
            name='go_straight_act_node',
            output='screen'
        ),

        # IncludeLaunchDescription(
        #     PythonLaunchDescriptionSource(
        #         os.path.join(camera_launch_dir,'azhsxf_camera.launch.py')
        #     ))
        # ,
        # IncludeLaunchDescription(
        #     PythonLaunchDescriptionSource(
        #         os.path.join(nav2_launch_dir,'wheeltec_nav2.launch.py')
        #     ))
])
