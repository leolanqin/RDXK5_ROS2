from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
import os

def generate_launch_description():

    # 获取 camera_bringup 包中的 launch 路径
    usbcam_dir = get_package_share_directory('camera_bringup')
    usbcam_launch_dir = os.path.join(usbcam_dir, 'launch')

    # 声明参数（如你希望从外部传 video_device，可用这个）
    declare_video_device_arg = DeclareLaunchArgument(
        'video_device', default_value='/dev/video0',
        description='USB 摄像头设备路径'
    )

    return LaunchDescription([
        declare_video_device_arg,

        # 1. 启动 USB 摄像头节点（使用 demo.launch.py）
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                os.path.join(usbcam_launch_dir, 'demo.launch.py')
            ),
            launch_arguments={
                'video_device': '/dev/RgbCam',
                'namespace': 'back_usbcam'
            }.items()
        ),

        # 2. 启动 YOLO 检测节点（你自己的节点）
        Node(
            package='yolo_detector',
            executable='back_yolo_node',
            name='back_yolo_node',
            output='screen',
            parameters=[]  # 可添加模型路径、类别等
        )
    ])
