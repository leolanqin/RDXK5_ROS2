import os

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory


def generate_launch_description():

    # 声明参数：video_device
    video_device_arg = DeclareLaunchArgument(
        'video_device',
        default_value='/dev/video0',
        description='路径：摄像头设备（如 /dev/video0、/dev/video1 等）'
    )

    cam_namespace=DeclareLaunchArgument(
        'namespace', 
        default_value='')

    # 使用 LaunchConfiguration 来获取动态传入的参数值
    video_device = LaunchConfiguration('video_device')

    # 获取 usb_cam 包的 share 路径
    usb_cam_dir = get_package_share_directory('camera_bringup')

    # 拼接参数文件路径
    params_path = os.path.join(
        usb_cam_dir,
        'config',
        'params.yaml'  # 确保此文件存在
    )

    # 创建启动描述对象
    ld = LaunchDescription()

    # 加入参数声明
    ld.add_action(video_device_arg)
    ld.add_action(cam_namespace)

    # 创建并添加摄像头节点
    ld.add_action(
        Node(
            package='usb_cam',
            executable='usb_cam_node_exe',
            name='usb_cam',
            output='screen',
            namespace=LaunchConfiguration('namespace'),
            parameters=[params_path, {'video_device': video_device}]
        )
    )

    return ld


