#!/usr/bin/env python3
# cd ~/dev_ws
# colcon build --packages-select phase_rtabmap_ros2
# . install/setup.bash
# ros2 launch phase_rtabmap_ros2 phase_rtabmap_launch.py
# ros2 launch phase_rtabmap_ros2 phase_rtabmap_launch.py left_serial:=40266661 right_serial:=40298125 camera_name:=746974616e24324 device_type:=titania interface_type:=usb exposure:=10000

from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import IncludeLaunchDescription, GroupAction
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.actions import DeclareLaunchArgument, ExecuteProcess
from launch.substitutions import LaunchConfiguration
from launch.substitutions import PathJoinSubstitution, TextSubstitution
from launch_ros.substitutions import FindPackageShare
from launch_ros.actions import PushRosNamespace

def generate_launch_description():

    # args that can be set from the command line or a default will be used
    camera_ns_launch_arg = DeclareLaunchArgument(
        "camera_ns", default_value=TextSubstitution(text="camera2")
    )
    rosbag_name_launch_arg = DeclareLaunchArgument(
        "rosbag_name", default_value=TextSubstitution(text="rosbag2_2024_03_25-14_50_59")
    )

    camera_ns_arg = LaunchConfiguration("camera_ns")
    rosbag_name_arg = LaunchConfiguration("rosbag_name")

    rosbag_read = ExecuteProcess(
    cmd=["ros2", "bag", "play", rosbag_name_arg],
    output="screen",
    )

    phase_camera = Node(
        package='phase_rtabmap_ros2',
        namespace=camera_ns_arg,
        executable='phase_read',
        output="screen",
        # remappings=[
        #     ('/left/image_raw', '/camera2/left/image_raw'),
        #     ('/right/image_raw', '/camera2/right/image_raw'),
        # ]
    )
    tf2 = Node(
        package='tf2_ros',
        executable="static_transform_publisher",
        name="camera_base_link",
        arguments = ["0", "0", "0", "-1.5707963267948966", "0", "-1.5707963267948966", "base_link", "camera_link"]
    )
    launch_rtabmap = GroupAction([
        PushRosNamespace(
            namespace=camera_ns_arg),
        IncludeLaunchDescription(
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
            'camera_info_topic': '/left/camera_info'
        }.items()
    )
    ])
    return LaunchDescription([
        camera_ns_launch_arg,
        rosbag_name_launch_arg,
        rosbag_read,
        phase_camera,
        tf2,
        launch_rtabmap,
    ])