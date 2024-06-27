#!/usr/bin/env python3
# cd ~/dev_ws
# colcon build --packages-select phase_rtabmap_ros2
# . install/setup.bash
# ros2 launch phase_rtabmap_ros2 phase_rtabmap_launch.py
# ros2 launch phase_rtabmap_ros2 phase_rtabmap_launch.py left_serial:=40266661 right_serial:=40298125 camera_name:=746974616e24324 device_type:=titania interface_type:=usb exposure:=10000

from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch.substitutions import PathJoinSubstitution, TextSubstitution
from launch_ros.substitutions import FindPackageShare

def generate_launch_description():

    # args that can be set from the command line or a default will be used
    input_folder_launch_arg = DeclareLaunchArgument(
        "input", default_value=TextSubstitution(text="/home/i3dr/Downloads/cam3/2024062610/")
    )

    input_arg = LaunchConfiguration("input")

    
    load_img = Node(
        package='phase_rtabmap_ros2',
        executable='img_pub',
        output="screen",
        arguments=[
            "--input", input_arg,
            ],
    )

    # ros2 run tf2_ros static_transform_publisher "0 0 0 1.5707963267948966 0 -1.5707963267948966 base_link camera_link
    tf2 = Node(
        package='tf2_ros',
        executable="static_transform_publisher",
        name="image_base_link",
        arguments = ["0", "0", "0", "1.5707963267948966", "1.5707963267948966", "-1.5707963267948966", "base_link", "image_link"]
    )

    # ros2 launch rtabmap_ros rtabmap.launch.py args:=--delete_db_on_start frame_id:=base_link rgb_topic:=/left/image_rect_color depth_topic:=/depth/image camera_info_topic:=/left/camera_info
    launch_rtabmap = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            PathJoinSubstitution([
                FindPackageShare('rtabmap_launch'),
                'launch/rtabmap.launch.py'
            ])
        ]),
        launch_arguments={
            'args': '--delete_db_on_start',
            'frame_id': 'base_link',
            'rgb_topic': '/left/image_rect',
            'depth_topic': '/depth/image',
            'camera_info_topic': '/left/camera_info',
        }.items()
    )
    return LaunchDescription([
        input_folder_launch_arg,
        load_img,
        tf2,
        launch_rtabmap,
    ])