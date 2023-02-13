# cd ~/dev_ws/install/phase_rtabmap_foxy/share/phase_rtabmap_foxy/
# ros2 launch phase_camera_launch.py

from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.actions import GroupAction
from launch_ros.actions import PushRosNamespace
import os

def generate_launch_description():
    
    phase_camera = Node(
        package='phase_rtabmap_foxy',
        namespace='stereo',
        executable='phase_camera',
        output="screen",
        remappings=[
            ('/stereo/left/image_raw', '/stereo_camera/left/image_raw'),
            ('/stereo/right/image_raw', '/stereo_camera/right/image_raw'),
            ('/stereo/left/image_rect', '/stereo_camera/left/image_rect'),
            ('/stereo/right/image_rect', '/stereo_camera/right/image_rect'),
            ('/stereo/left/camera_info', '/stereo_camera/left/camera_info'),
            ('/stereo/right/camera_info', '/stereo_camera/right/camera_info'),
        ]
    )
    return LaunchDescription([
        phase_camera,
    ])