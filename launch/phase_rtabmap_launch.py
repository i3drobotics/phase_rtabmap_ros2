#!/usr/bin/env python3
# cd ~/dev_ws
# colcon build --packages-select phase_rtabmap_foxy
# . install/setup.bash
# cd ~/dev_ws/install/phase_rtabmap_foxy/share/phase_rtabmap_foxy/
# ros2 launch phase_rtabmap_launch.py
# ros2 launch phase_rtabmap_launch.py left_serial:=40266661 right_serial:=40298125 camera_name:=746974616e24324 device_type:=titania interface_type:= usb exposure:=10000

from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch.substitutions import TextSubstitution
import os

def generate_launch_description():

    print("\n")
    print("\n")
    print("\n")
    print("*********************Current path********************")
    print(os.path.dirname(os.path.realpath(__file__)))
    print("*********************Current path********************")
    print("\n")
    print("\n")
    print("\n")

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
    device_type_launch_arg = DeclareLaunchArgument(
        "device_type", default_value=TextSubstitution(text="phobos")
    )
    interface_type_launch_arg = DeclareLaunchArgument(
        "interface_type", default_value=TextSubstitution(text="usb")
    )
    exposure_launch_arg = DeclareLaunchArgument(
        "exposure", default_value=TextSubstitution(text="25000")
    )

    left_serial_arg = LaunchConfiguration("left_serial")
    right_serial_arg = LaunchConfiguration("right_serial")
    camera_name_arg = LaunchConfiguration("camera_name")
    device_type_arg = LaunchConfiguration("device_type")
    interface_type_arg = LaunchConfiguration("interface_type")
    exposure_arg = LaunchConfiguration("exposure")

    
    phase_camera = Node(
        package='phase_rtabmap_foxy',
        executable='phase_camera',
        name='phase_pub',
        output="screen",
        arguments=[
            "--left_serial", left_serial_arg,
            "--right_serial", right_serial_arg,
            "--camera_name", camera_name_arg,
            "--device_type", device_type_arg,
            "--interface_type", interface_type_arg, 
            "--exposure", exposure_arg
            ],
    )
    launch_stereo_image_proc = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
                'stereo_image_proc.launch.py'
        ]),
    )
    tf2 = Node(
        package='tf2_ros',
        executable="static_transform_publisher",
        name="camera_base_link",
        arguments = ["0", "0", "0", "-1.5707963267948966", "0", "-1.5707963267948966", "base_link", "camera_link"]
    )
    launch_rtabmap = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
                'rtabmap.launch.py'
        ]),
        launch_arguments={
            'args': '--delete_db_on_start',
            'frame_id': 'base_link',
            'rgb_topic': '/left/image_rect_color',
            'depth_topic': '/depth/image',
            'camera_info_topic': '/left/camera_info'
        }.items()
    )
    return LaunchDescription([
        left_serial_launch_arg,
        right_serial_launch_arg,
        camera_name_launch_arg,
        device_type_launch_arg,
        interface_type_launch_arg,
        exposure_launch_arg,
        phase_camera,
        launch_stereo_image_proc,
        tf2,
        launch_rtabmap,
    ])