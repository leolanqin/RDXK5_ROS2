import os
# from pathlib import Path
# import launch_ros.actions
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import (DeclareLaunchArgument, GroupAction,
                            IncludeLaunchDescription, SetEnvironmentVariable)
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.launch_description_sources import AnyLaunchDescriptionSource

def generate_launch_description():
	astra_dir = get_package_share_directory('astra_camera')
	astra_launch_dir = os.path.join(astra_dir,'launch')
 
	usbcam_dir=get_package_share_directory('camera_bringup')
	usbcam_launch_dir = os.path.join(usbcam_dir,'launch')

	usbcam_arg = DeclareLaunchArgument(
		'video_device', default_value='/dev/video0',
		description='video device serial number.')
 
	# 这是 launch 库提供的通用 Launch 文件源加载器，能加载 .launch.py、.launch.xml 等类型的 Launch 文件
	Astra_S = IncludeLaunchDescription(
    AnyLaunchDescriptionSource(
        os.path.join(astra_launch_dir, 'astra.launch.xml')
    ),
    launch_arguments={
        'camera_name': 'former_astra'  # 这里可以修改为你想要的相机名称
    }.items()
)
	#Select the UVC device {Rgbcam(C70)、Astra_Gemini、Astra_Dabai}
	azhsxf_Usbcam = IncludeLaunchDescription(
	PythonLaunchDescriptionSource(
		os.path.join(usbcam_launch_dir,'demo.launch.py')
	),
	launch_arguments={
		'video_device': '/dev/RgbCam','namespace': 'back_usbcam'
		}.items())

	# Create the launch description and populate
	ld = LaunchDescription()
	
	#Select your camera here, options include:
	#Astra_S、Astra_Pro、Dabai、Gemini、Wheeltec_Usbcam
	ld.add_action(Astra_S)
	ld.add_action(azhsxf_Usbcam)
	ld.add_action(usbcam_arg)

	return ld
