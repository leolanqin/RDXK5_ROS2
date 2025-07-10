from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import AnyLaunchDescriptionSource
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
import os

def generate_launch_description():

    astra_dir = get_package_share_directory('astra_camera')
    astra_launch_dir = os.path.join(astra_dir, 'launch')

    return LaunchDescription([

        # 1. 启动 Astra 相机
        IncludeLaunchDescription(
            AnyLaunchDescriptionSource(
                os.path.join(astra_launch_dir, 'astra.launch.xml')
            ),
            launch_arguments={
                'camera_name': 'former_astra'  # 自定义相机名称
            }.items()
        ),

        # 2. 启动 YOLO 检测节点
        Node(
            package='yolo_detector',
            executable='former_yolo_node',
            name='former_yolo_node',
            output='screen',
            parameters=[]  # 如有参数请填写
        )
    ])
