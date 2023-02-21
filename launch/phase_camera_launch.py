# cd ~/dev_ws/install/phase_rtabmap_foxy/share/phase_rtabmap_foxy/
# ros2 launch phase_camera_launch.py

from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.actions import GroupAction
from launch_ros.actions import PushRosNamespace
import os
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch.substitutions import TextSubstitution

def generate_launch_description():

        # args that can be set from the command line or a default will be used
    left_serial_launch_arg = DeclareLaunchArgument(
        "left_serial", default_value=TextSubstitution(text="23638717")
    )
    right_serial_launch_arg = DeclareLaunchArgument(
        "right_serial", default_value=TextSubstitution(text="23638711")
    )
    camera_name_launch_arg = DeclareLaunchArgument(
        "camera_name", default_value=TextSubstitution(text="Basler acA2440-35uc")
    )
    camera_type_launch_arg = DeclareLaunchArgument(
        "camera_type", default_value=TextSubstitution(text="phobos")
    )
    exposure_launch_arg = DeclareLaunchArgument(
        "exposure", default_value=TextSubstitution(text="25000")
    )

    left_serial_arg = LaunchConfiguration("left_serial")
    right_serial_arg = LaunchConfiguration("right_serial")
    camera_name_arg = LaunchConfiguration("camera_name")
    camera_type_arg = LaunchConfiguration("camera_type")
    exposure_arg = LaunchConfiguration("exposure")
    
    phase_camera = Node(
        package='phase_rtabmap_foxy',
        namespace='stereo',
        executable='phase_camera',
        output="screen",
        arguments=[
            "--left_serial", left_serial_arg,
            "--right_serial", right_serial_arg,
            "--camera_name", camera_name_arg,
            "--camera_type", camera_type_arg, 
            "--exposure", exposure_arg
            ],
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
        left_serial_launch_arg,
        right_serial_launch_arg,
        camera_name_launch_arg,
        camera_type_launch_arg,
        exposure_launch_arg,
        phase_camera,
    ])